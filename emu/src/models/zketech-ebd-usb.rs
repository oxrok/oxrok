// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! A ZKETECH EBD-USB+ electronic load on the CDC-ACM shell: frames that start
//! with `fa` and end with `f8`, ten bytes from the host and nineteen from the
//! device, which streams its measurements once told to.
//!
//! The device, and how sure this is:
//!
//! - The USB side: the sigrok wiki's `lsusb -v` shows a PL2303, `067b:2303`,
//!   "Prolific Technology Inc." "USB-Serial Controller D". This model is NOT
//!   that: `pl2303` would take those ids and the tools would meet another
//!   kernel driver, so it is a CDC-ACM function with the shell's descriptors
//!   and test ids, an artifact of the emulation. The tools see a tty either
//!   way.
//! - The line: 9600/8e1, from the QB64 program of mdijkens (a gist, "ZKE Tech
//!   EBD+"), written against a real unit. The wiki names no line.
//! - The frames: the wiki's protocol section. Byte 1 is the type, two-byte
//!   values are base 240, the last byte but one is the XOR of all but the first
//!   and the last. The unit tests hold this model to the wiki's examples.
//!   - From the host: `05` starts measurements, `06` stops them, `01` starts
//!     the load, `02` toggles it, `07` updates it; the last two with the
//!     current limit in mA at 2, the cutoff in 10 mV at 4 and a time limit in
//!     minutes at 6, which the device does not enforce.
//!   - From the device: `00` a measurement without load, `0a` with: current in
//!     0.1 mA at 2, volts in mV at 4, D+ at 6, D- at 8, the limit at 10, the
//!     cutoff at 12, minutes at 14, the device at 16, `1a` for the EBD-USB+.
//!   - Also from the device: `64` and `6e`, the same without and with load, but
//!     with a version and two values the wiki calls constant at 10 to 15.
//!     `device=1` sends those instead.
//! - The ranges, 0 to 4 A and 0 to 21 V: the wiki.
//!
//! What is an inference, as neither source says:
//!
//! - What the checksum is when the XOR reaches `f0`. The wiki says no byte but
//!   the first and last can be `fa` or `f8`, and the gist's host takes its XOR
//!   modulo `f0`, which no example of the wiki shows. So it sends that modulo,
//!   and `raw-checksum=1` the plain XOR, which then can be `fa` or `f8`. From the
//!   host it takes both: the modulo, as the gist's host works against a real one,
//!   and the plain XOR the driver sends, as nothing shows a real one refusing
//!   that. It takes a frame by its length, so even a checksum of `f8` is no end.
//! - That `06` leaves the load as it is. A program stopping a load sends `02`
//!   first, in the driver and in the gist.
//! - The rate of the stream. Neither says, and the driver gives each frame at
//!   most some 2 s (200 reads, 11 ms each).
//! - That a load falls off below its cutoff, which is what `sag=` shows. The
//!   name is all there is for it.
//! - That the current is exactly the limit, and that the volts ripple: there
//!   only so that each frame of a capture differs.
//!
//! Emulation artifacts:
//!
//! - A first frame goes out with the `05`'s last byte, and later ones after
//!   `period=` offers of a read, counted from the last frame or command, and
//!   never while a command is coming in. So a `.tx` does not depend on timing:
//!   the driver answers a frame within a period, and its 10 ms between bytes
//!   do not let a frame in between them. `chunk=1` makes offers come fast, and a
//!   period short with them: use a long `period=` there.
//! - No byte is lost, where the wiki says a real one loses them in both
//!   directions; `replies=` plays what a host would then see.
//!
//! Not modelled: `04` calibrates, which no driver path sends and is flagged;
//! the time limit, which the driver leaves at 0; the wiki's load timer frame,
//! other than through `replies=`.
//!
//! Options, besides the shell's ([`Faults`]):
//!
//! - `vbus=MV`, `dp=MV`, `dm=MV`, `limit=MA`, `cutoff=TEN_MV`, the state at
//!   power on, by default the wiki's example; `load=1` starts with it on;
//! - `sag=MV`, volts lost per frame under load;
//! - `device=1`, frames of type `64` and `6e`;
//! - `raw-checksum=1`, see above;
//! - `period=N`, offers of a read per frame;
//! - `frames=N`, frames per power on, a scan's too, then silence and the event
//!   point a period later, with commands still taken, as the wiki's "sometimes it doesn't
//!   send any messages anymore";
//! - `replies=HEX[,HEX...]`, frames played in turn instead of the computed one,
//!   of any length up to 256;
//! - `deaf=1`, which loses every load command, as the wiki says a real one can;
//! - `setters=0`, which flags any load command, for what must change nothing;
//! - `uart=LINE`.
//!
//! What no case reaches in the driver, with `--gcov`: the setters past their
//! "later" (`protocol.c:417-439,473-495`), as they need a measurement seen and
//! neither sigrok-cli nor the library runner sets anything in a capture; the end
//! of `ebd_loadstart` (118-123), as `send_cmd` never returns `SR_OK`; a read's
//! error (192-193, 212-213), as a read after an unplug gives none and each turn
//! then takes 1 s (`serial.c:1159`), 200 s in all;
//! and the checks on NULLs, unknown keys and a missing `devc`.
//!
//! The pedantic checks: the ends, the checksum, the type, zeros where a frame
//! has no value, a time limit other than 0, values not in base 240 or out of
//! range, and nothing left of a frame when a tool opens.

use crate::model::DeviceModel;
use crate::options::{parse_hex_list, parse_number};
use crate::shells::cdc::{Cdc, Faults, Identity, Line, LineEvent, Port, Serial};
use crate::shells::hidcable::Packets;
use crate::{Error, Result};
use std::fmt::Write as _;

/// The model name.
pub const NAME: &str = "zketech-ebd-usb";
/// What `oxrok-emu list` says about it.
pub const ABOUT: &str = "A ZKETECH EBD-USB+ electronic load on CDC-ACM; vbus=, dp=, dm=, limit=, \
                         cutoff=, load=, sag=, device=, raw-checksum=, period=, frames=, replies=, deaf=, \
                         setters=, uart=, and the shell's stall=, chunk=, in-limit=, in-stall=, \
                         out-hold=";

const BEGIN: u8 = 0xfa;
const END: u8 = 0xf8;
/// A command's length.
const COMMAND: usize = 10;
/// A measurement's length.
const MEASUREMENT: usize = 19;
/// The largest byte of a value in base 240.
const BASE: u32 = 240;
/// The same, for a byte.
const BASE_BYTE: u8 = 0xf0;
const LIMIT_MAX: u32 = 4000;
const CUTOFF_MAX: u32 = 2100;
const VOLTS_MAX: u32 = 21000;
/// The device byte of an EBD-USB+.
const EBD_USB_PLUS: u8 = 0x1a;
/// `replies=` may be longer than a frame, up to this.
const LONGEST_REPLY: usize = 256;
/// About 300 ms, and the stream's frames stay well inside what the driver
/// waits for one: see the module's note on the rate.
const PERIOD: u32 = 15;

/// A load from the `-o` options.
///
/// # Errors
///
/// On an unknown option or a value out of range.
pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>> {
    Ok(Box::new(function(options)?))
}

fn function(options: &[(String, String)]) -> Result<Cdc<Load>> {
    let mut faults = Faults::default();
    let mut uart = Line::parse("9600/8e1")?;
    let mut config = Config::default();
    for (k, v) in options {
        if faults.option(k, v)? {
            continue;
        }
        match k.as_str() {
            "vbus" => config.vbus = parse_number(k, v, 0..=VOLTS_MAX)?,
            "dp" => config.dp = parse_number(k, v, 0..=VOLTS_MAX)?,
            "dm" => config.dm = parse_number(k, v, 0..=VOLTS_MAX)?,
            "limit" => config.limit = parse_number(k, v, 0..=LIMIT_MAX)?,
            "cutoff" => config.cutoff = parse_number(k, v, 0..=CUTOFF_MAX)?,
            "load" => config.load = parse_number(k, v, 0..=1)? == 1,
            "sag" => config.sag = parse_number(k, v, 0..=VOLTS_MAX)?,
            "device" => config.device = parse_number(k, v, 0..=1)? == 1,
            "raw-checksum" => config.raw_checksum = parse_number(k, v, 0..=1)? == 1,
            "period" => config.period = parse_number(k, v, 1..=100_000)?,
            "frames" => config.frames = Some(parse_number(k, v, 0..=100_000)?),
            "replies" => {
                config.replies = Some(Packets::new(parse_hex_list(k, v, 1, LONGEST_REPLY)?));
            }
            "deaf" => config.deaf = parse_number(k, v, 0..=1)? == 1,
            "setters" => config.setters = parse_number(k, v, 0..=1)? == 1,
            "uart" => uart = Line::parse(v)?,
            other => return Err(Error::Invalid(format!("{NAME} has no option {other}"))),
        }
    }
    // Test ids: see the module's note on the real PL2303.
    let id = Identity::new(0x1209, 0x0001, "oxrok-emu", "ZKETECH EBD-USB+ stand-in");
    Ok(Cdc::new(NAME, &id, faults, Load::new(config)).behind_uart(uart))
}

#[derive(Clone, Debug)]
#[allow(clippy::struct_excessive_bools)]
struct Config {
    vbus: u32,
    dp: u32,
    dm: u32,
    limit: u32,
    cutoff: u32,
    load: bool,
    sag: u32,
    device: bool,
    raw_checksum: bool,
    period: u32,
    frames: Option<u32>,
    replies: Option<Packets>,
    deaf: bool,
    setters: bool,
}

impl Default for Config {
    /// The wiki's example of a measurement without load.
    fn default() -> Config {
        Config {
            vbus: 4997,
            dp: 1849,
            dm: 1853,
            limit: 100,
            cutoff: 10,
            load: false,
            sag: 0,
            device: false,
            raw_checksum: false,
            period: PERIOD,
            frames: None,
            replies: None,
            deaf: false,
            setters: true,
        }
    }
}

struct Load {
    config: Config,
    replies: Option<Packets>,
    /// What has come of a command so far, shorter than one.
    pending: Vec<u8>,
    streaming: bool,
    on: bool,
    limit: u32,
    cutoff: u32,
    /// Millivolts lost so far under `sag=`.
    lost: u32,
    /// Frames sent since power on.
    sent: u32,
    /// Offers since the last frame or command.
    offers: u32,
    marked: bool,
}

/// `value` in base 240, in two bytes.
fn base240(value: u32) -> [u8; 2] {
    let hi = value / BASE;
    let lo = value % BASE;
    [u8::try_from(hi).expect("below 240 * 256"), u8::try_from(lo).expect("below 240")]
}

fn from_base240(hi: u8, lo: u8) -> Option<u32> {
    (u32::from(hi) < BASE && u32::from(lo) < BASE).then(|| u32::from(hi) * BASE + u32::from(lo))
}

fn xor(bytes: &[u8]) -> u8 {
    bytes.iter().fold(0, |x, b| x ^ b)
}

fn show(bytes: &[u8]) -> String {
    bytes.iter().fold(String::new(), |mut s, b| {
        let _ = write!(s, "{b:02x}");
        s
    })
}

impl Load {
    fn new(config: Config) -> Load {
        Load {
            replies: config.replies.clone(),
            pending: Vec::new(),
            streaming: false,
            on: config.load,
            limit: config.limit,
            cutoff: config.cutoff,
            lost: 0,
            sent: 0,
            offers: 0,
            marked: false,
            config,
        }
    }

    /// What is measured now. The volts ripple, so that frames differ.
    fn volts(&self) -> u32 {
        (self.config.vbus + self.sent % 8).saturating_sub(self.lost).min(VOLTS_MAX)
    }

    fn measurement(&self) -> Vec<u8> {
        let kind = match (self.config.device, self.on) {
            (false, false) => 0x00,
            (false, true) => 0x0a,
            (true, false) => 0x64,
            (true, true) => 0x6e,
        };
        // In 0.1 mA.
        let current = if self.on { self.limit * 10 } else { 0 };
        let mut f = vec![0; MEASUREMENT];
        f[0] = BEGIN;
        f[1] = kind;
        f[2..4].copy_from_slice(&base240(current));
        f[4..6].copy_from_slice(&base240(self.volts()));
        f[6..8].copy_from_slice(&base240(self.config.dp));
        f[8..10].copy_from_slice(&base240(self.config.dm));
        if self.config.device {
            // The wiki's version and constants.
            f[10..16].copy_from_slice(&[0x01, 0x50, 0x09, 0x6f, 0x10, 0xa0]);
        } else {
            f[10..12].copy_from_slice(&base240(self.limit));
            f[12..14].copy_from_slice(&base240(self.cutoff));
        }
        f[16] = EBD_USB_PLUS;
        let checksum = xor(&f[1..17]);
        f[17] = if self.config.raw_checksum { checksum } else { checksum % BASE_BYTE };
        f[18] = END;
        f
    }

    /// The next frame, if one is left to send.
    fn send_frame(&mut self, port: &mut Port<'_>) {
        self.offers = 0;
        if self.config.frames.is_some_and(|n| self.sent >= n) {
            return;
        }
        let frame = match &mut self.replies {
            Some(list) => list.take(),
            None => self.measurement(),
        };
        port.send(&frame);
        self.sent += 1;
        if self.on {
            self.lost = self.lost.saturating_add(self.config.sag);
            if self.config.vbus.saturating_sub(self.lost) < self.cutoff * 10 {
                self.on = false;
            }
        }
    }

    /// A command's two values, when they hold.
    fn values(f: &[u8], port: &mut Port<'_>) -> Option<(u32, u32)> {
        let (Some(limit), Some(cutoff)) = (from_base240(f[2], f[3]), from_base240(f[4], f[5]))
        else {
            port.violation(format!("{}: a value not in base 240", show(f)));
            return None;
        };
        if limit > LIMIT_MAX {
            port.violation(format!("{}: over 4 A", show(f)));
            return None;
        }
        if cutoff > CUTOFF_MAX {
            port.violation(format!("{}: over 21 V", show(f)));
            return None;
        }
        if f[6..8] != [0, 0] {
            port.violation(format!("{}: a time limit, which no driver sets", show(f)));
            return None;
        }
        Some((limit, cutoff))
    }

    fn command(&mut self, f: &[u8], port: &mut Port<'_>) {
        if f[0] != BEGIN || f[9] != END {
            port.violation(format!("{}: no frame", show(f)));
            return;
        }
        let checksum = xor(&f[1..8]);
        if f[8] != checksum && f[8] != checksum % BASE_BYTE {
            port.violation(format!("{}: wrong checksum", show(f)));
            return;
        }
        self.offers = 0;
        let is_load = matches!(f[1], 0x01 | 0x02 | 0x07);
        if is_load && !self.config.setters {
            port.violation(format!("{}: a load command no one asked for", show(f)));
        }
        match f[1] {
            0x01 | 0x07 => {
                if let Some((limit, cutoff)) = Self::values(f, port)
                    && !self.config.deaf
                {
                    self.limit = limit;
                    self.cutoff = cutoff;
                    if f[1] == 0x01 {
                        self.on = true;
                    }
                }
            }
            0x02 | 0x05 | 0x06 => {
                if f[2..8].iter().any(|b| *b != 0) {
                    port.violation(format!("{}: values in a command without any", show(f)));
                    return;
                }
                match f[1] {
                    0x02 if !self.config.deaf => self.on = !self.on,
                    0x02 => {}
                    0x05 => {
                        self.streaming = true;
                        self.send_frame(port);
                    }
                    _ => self.streaming = false,
                }
            }
            _ => port.violation(format!("{}: no such command", show(f))),
        }
    }
}

impl Serial for Load {
    fn received(&mut self, data: &[u8], port: &mut Port<'_>) {
        // Bounded: what is left of `pending` after the loop is under a frame.
        for chunk in data.chunks(COMMAND) {
            self.pending.extend_from_slice(chunk);
            while self.pending.len() >= COMMAND {
                let frame: Vec<u8> = self.pending.drain(..COMMAND).collect();
                self.command(&frame, port);
            }
        }
    }

    fn changed(&mut self, event: LineEvent, port: &mut Port<'_>) {
        if event.opened() && !self.pending.is_empty() {
            port.violation(format!("{}: a command left unfinished", show(&self.pending)));
            self.pending.clear();
        }
    }

    fn tick(&mut self, port: &mut Port<'_>) {
        if !self.streaming || !self.pending.is_empty() {
            return;
        }
        self.offers = self.offers.saturating_add(1);
        if self.offers < self.config.period {
            return;
        }
        if self.config.frames.is_none_or(|n| self.sent < n) {
            self.send_frame(port);
        } else if !self.marked {
            self.marked = true;
            port.mark();
        }
    }

    fn reset(&mut self) {
        *self = Load::new(self.config.clone());
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::shells::cdc::host::Host;

    fn load(options: &[(&str, &str)]) -> Host<Load> {
        let o: Vec<(String, String)> =
            options.iter().map(|(k, v)| ((*k).to_owned(), (*v).to_owned())).collect();
        Host::open(function(&o).unwrap(), Line::parse("9600/8e1").unwrap(), true, true)
    }

    fn command(kind: u8, limit: u32, cutoff: u32) -> Vec<u8> {
        let mut f = vec![0; COMMAND];
        f[0] = BEGIN;
        f[1] = kind;
        f[2..4].copy_from_slice(&base240(limit));
        f[4..6].copy_from_slice(&base240(cutoff));
        f[8] = xor(&f[1..8]);
        f[9] = END;
        f
    }

    fn start() -> Vec<u8> {
        command(0x05, 0, 0)
    }

    fn violations(h: &mut Host<Load>) -> usize {
        h.v.drain().len()
    }

    /// Offers until a frame comes, with a bound.
    fn next_frame(h: &mut Host<Load>) -> Vec<u8> {
        for _ in 0..1000 {
            let got = h.read();
            if !got.is_empty() {
                return got;
            }
        }
        Vec::new()
    }

    #[test]
    fn the_wikis_commands_are_good_frames() {
        for frame in [
            "fa0500000000000005f8",
            "fa0600000000000006f8",
            "fa0200000000000002f8",
            "fa010064000a00006ff8",
            "fa070064000a000069f8",
        ] {
            let hex = parse_hex_list("x", frame, 1, 16).unwrap().remove(0);
            let mut h = load(&[]);
            h.write(&hex);
            assert_eq!(violations(&mut h), 0, "{frame}");
        }
    }

    #[test]
    fn a_start_answers_with_the_wikis_measurement_without_load() {
        let mut h = load(&[]);
        h.write(&start());
        assert_eq!(show(&h.read()), "fa00000014c507a907ad0064000a00001aa1f8");
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn device_frames_have_the_wikis_constants() {
        let mut h = load(&[("device", "1"), ("vbus", "0"), ("dp", "299"), ("dm", "231")]);
        h.write(&start());
        assert_eq!(show(&h.read()), "fa6400000000013b00e70150096f10a01a24f8");
    }

    #[test]
    fn the_stream_comes_by_offers_and_stops() {
        let mut h = load(&[("period", "3")]);
        h.write(&start());
        assert_eq!(h.read().len(), MEASUREMENT);
        // That read's last offer was the first of the period.
        assert_eq!(h.read(), b"", "offer 2");
        assert_eq!(h.read().len(), MEASUREMENT, "offer 3, and offer 1 of the next");
        h.write(&command(0x06, 0, 0));
        for _ in 0..10 {
            assert_eq!(h.read(), b"");
        }
    }

    #[test]
    fn nothing_streams_before_a_start() {
        let mut h = load(&[("period", "2")]);
        for _ in 0..10 {
            assert_eq!(h.read(), b"");
        }
    }

    #[test]
    fn no_frame_comes_while_a_command_is_coming_in() {
        let mut h = load(&[("period", "2")]);
        h.write(&start());
        assert_eq!(h.read().len(), MEASUREMENT);
        h.write(&command(0x01, 100, 10)[..3]);
        for _ in 0..10 {
            assert_eq!(h.read(), b"");
        }
        h.write(&command(0x01, 100, 10)[3..]);
        assert_eq!(next_frame(&mut h)[1], 0x0a);
    }

    #[test]
    fn the_load_goes_on_takes_changes_and_toggles() {
        let mut h = load(&[("period", "2")]);
        h.write(&start());
        assert_eq!(h.read()[1], 0);
        h.write(&command(0x01, 1000, 300));
        let f = next_frame(&mut h);
        assert_eq!(f[1], 0x0a);
        assert_eq!(from_base240(f[2], f[3]), Some(10_000), "1 A in 0.1 mA");
        assert_eq!(from_base240(f[10], f[11]), Some(1000));
        assert_eq!(from_base240(f[12], f[13]), Some(300));
        h.write(&command(0x07, 500, 200));
        let f = next_frame(&mut h);
        assert_eq!(f[1], 0x0a);
        assert_eq!(from_base240(f[10], f[11]), Some(500));
        h.write(&command(0x02, 0, 0));
        assert_eq!(next_frame(&mut h)[1], 0);
        h.write(&command(0x02, 0, 0));
        assert_eq!(next_frame(&mut h)[1], 0x0a);
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn a_stop_leaves_the_load_on() {
        let mut h = load(&[]);
        h.write(&start());
        h.read();
        h.write(&command(0x01, 1000, 300));
        h.write(&command(0x06, 0, 0));
        h.write(&start());
        assert_eq!(h.read()[1], 0x0a);
    }

    #[test]
    fn a_sagging_load_falls_off_at_its_cutoff() {
        let mut h = load(&[("period", "2"), ("sag", "2000"), ("vbus", "5000"), ("load", "1"), ("cutoff", "200")]);
        h.write(&start());
        assert_eq!(h.read()[1], 0x0a, "5 V");
        assert_eq!(next_frame(&mut h)[1], 0x0a, "3 V");
        assert_eq!(next_frame(&mut h)[1], 0, "1 V is below 2 V");
    }

    #[test]
    fn frames_runs_out_and_marks_at_an_offer() {
        let mut h = load(&[("period", "2"), ("frames", "1")]);
        h.write(&start());
        assert_eq!(h.read().len(), MEASUREMENT);
        assert_eq!(next_frame(&mut h), b"");
        assert!(h.cdc.mark());
        h.write(&command(0x01, 100, 10));
        assert!(h.cdc.model.on, "a silent one still takes a command");
    }

    #[test]
    fn frames_0_is_silent_from_the_start() {
        let mut h = load(&[("frames", "0")]);
        h.write(&start());
        assert!(!h.cdc.mark(), "not inside the write");
        for _ in 0..PERIOD - 1 {
            assert_eq!(h.read(), b"");
            assert!(!h.cdc.mark(), "not before a period");
        }
        assert_eq!(h.read(), b"");
        assert!(h.cdc.mark());
    }

    #[test]
    fn deaf_loses_load_commands_and_not_the_others() {
        let mut h = load(&[("deaf", "1")]);
        h.write(&start());
        assert_eq!(h.read().len(), MEASUREMENT);
        h.write(&command(0x01, 1000, 300));
        assert!(!h.cdc.model.on);
        assert_eq!(h.cdc.model.limit, 100);
        h.write(&command(0x02, 0, 0));
        assert!(!h.cdc.model.on);
        h.write(&command(0x06, 0, 0));
        assert!(!h.cdc.model.streaming);
    }

    #[test]
    fn replies_play_in_turn() {
        let mut h = load(&[("replies", "fa00f8,fb"), ("period", "2")]);
        h.write(&start());
        assert_eq!(h.read(), [0xfa, 0x00, 0xf8]);
        assert_eq!(next_frame(&mut h), [0xfb]);
        assert_eq!(next_frame(&mut h), [0xfa, 0x00, 0xf8]);
    }

    #[test]
    fn each_check_fires() {
        let mut no_begin = start();
        no_begin[0] = 0x55;
        let mut no_end = start();
        no_end[9] = 0x55;
        let mut wrong_sum = start();
        wrong_sum[8] ^= 1;
        let mut values_in_a_start = command(0x05, 1, 0);
        values_in_a_start[8] = xor(&values_in_a_start[1..8]);
        let mut time_limit = command(0x01, 100, 10);
        time_limit[7] = 1;
        time_limit[8] = xor(&time_limit[1..8]);
        let mut hi_240 = command(0x01, 0, 0);
        hi_240[2] = 240;
        hi_240[8] = xor(&hi_240[1..8]);
        let mut lo_240 = command(0x07, 0, 0);
        lo_240[5] = 240;
        lo_240[8] = xor(&lo_240[1..8]);
        for bad in [
            no_begin,
            no_end,
            wrong_sum,
            values_in_a_start,
            time_limit,
            hi_240,
            lo_240,
            command(0x01, 4001, 0),
            command(0x07, 0, 2101),
            command(0x04, 0, 0),
        ] {
            let mut h = load(&[]);
            h.write(&bad);
            assert_eq!(violations(&mut h), 1, "{}", show(&bad));
            assert_eq!(h.read(), b"", "{}", show(&bad));
            assert!(!h.cdc.model.on);
            assert_eq!(h.cdc.model.limit, 100, "{}", show(&bad));
        }
    }

    #[test]
    fn the_limits_themselves_are_taken() {
        let mut h = load(&[]);
        h.write(&command(0x01, 4000, 2100));
        assert_eq!(violations(&mut h), 0);
        assert_eq!((h.cdc.model.limit, h.cdc.model.cutoff), (4000, 2100));
    }

    #[test]
    fn setters_0_flags_load_commands_and_not_the_others() {
        let mut h = load(&[("setters", "0")]);
        h.write(&start());
        h.write(&command(0x06, 0, 0));
        assert_eq!(violations(&mut h), 0);
        for kind in [0x01, 0x07] {
            h.write(&command(kind, 100, 10));
            assert_eq!(violations(&mut h), 1);
        }
        h.write(&command(0x02, 0, 0));
        assert_eq!(violations(&mut h), 1);
    }

    /// 128 mV: all eight ripples have an XOR of `f0` or more, so a host that checks
    /// the plain XOR takes none of them.
    #[test]
    fn some_readings_fail_the_plain_xor_all_through_the_ripple() {
        let mut h = load(&[("vbus", "128"), ("period", "2")]);
        h.write(&start());
        let mut frame = h.read();
        for _ in 0..8 {
            assert_ne!(xor(&frame[1..18]), 0, "{}", show(&frame));
            frame = next_frame(&mut h);
        }
    }

    #[test]
    fn a_hosts_checksum_modulo_f0_is_taken_too() {
        let mut f = command(0x01, 3824, 16);
        f[8] %= BASE_BYTE;
        assert_eq!(f[8], 0x0e);
        let mut h = load(&[]);
        h.write(&f);
        assert_eq!(violations(&mut h), 0);
        assert!(h.cdc.model.on);
    }

    /// A checksum that looks like the frame's end does not end it.
    #[test]
    fn a_checksum_of_f8_is_no_end() {
        let f = command(0x01, 3824, 22);
        assert_eq!(show(&f), "fa010fe000160000f8f8");
        let mut h = load(&[]);
        h.write(&f);
        assert_eq!(violations(&mut h), 0);
        assert_eq!(h.cdc.model.cutoff, 22);
    }

    #[test]
    fn a_checksum_at_the_top_is_taken() {
        // 3.824 A and 0.16 V: 01 ^ 0f ^ e0 ^ 00 ^ 10 is fe.
        let f = command(0x01, 3824, 16);
        assert_eq!(f[8], 0xfe);
        let mut h = load(&[]);
        h.write(&f);
        assert_eq!(violations(&mut h), 0);
        assert!(h.cdc.model.on);
    }

    /// 4956 mV makes the XOR `f8`.
    #[test]
    fn a_checksum_that_would_be_an_end() {
        let mut h = load(&[("vbus", "4956")]);
        h.write(&start());
        let f = h.read();
        assert_eq!((f[17], f[18]), (0x08, END), "modulo f0");
        let mut h = load(&[("vbus", "4956"), ("raw-checksum", "1")]);
        h.write(&start());
        let f = h.read();
        assert_eq!((f[17], f[18]), (END, END), "the plain XOR");
    }

    #[test]
    fn a_command_left_unfinished_shows_at_the_next_open() {
        let mut h = load(&[]);
        h.write(&[BEGIN, 0x05]);
        h.set_lines(false, false);
        assert_eq!(violations(&mut h), 0);
        h.set_lines(true, true);
        assert_eq!(violations(&mut h), 1);
        assert_eq!(h.cdc.model.pending, b"");
    }

    #[test]
    fn a_long_write_keeps_the_buffer_short() {
        let mut h = load(&[]);
        h.write(&[0x55; 4096]);
        assert!(h.cdc.model.pending.len() < COMMAND);
        assert_eq!(violations(&mut h), 4096 / COMMAND);
    }

    #[test]
    fn it_listens_at_its_line_only() {
        let mut h = load(&[]);
        h.set_line(Line::parse("9600/8n1").unwrap());
        h.write(&start());
        assert_eq!(h.read(), b"");
        let mut h = load(&[("uart", "19200/8e1")]);
        h.set_line(Line::parse("19200/8e1").unwrap());
        h.write(&start());
        assert_eq!(h.read().len(), MEASUREMENT);
    }

    #[test]
    fn a_reset_goes_back_to_power_on() {
        let mut h = load(&[("frames", "1"), ("replies", "aa,bb")]);
        h.write(&start());
        assert_eq!(h.read(), [0xaa]);
        h.write(&command(0x01, 1000, 300));
        h.cdc.model.reset();
        assert!(!h.cdc.model.on);
        assert_eq!(h.cdc.model.limit, 100);
        h.write(&start());
        assert_eq!(h.read(), [0xaa], "the list and the count start over");
    }

    #[test]
    fn options_parse() {
        for bad in [
            ("vbus", "21001"),
            ("limit", "4001"),
            ("cutoff", "2101"),
            ("load", "2"),
            ("device", "2"),
            ("period", "0"),
            ("replies", "abc"),
            ("deaf", "2"),
            ("setters", "2"),
            ("uart", "9600"),
            ("nothing", "1"),
        ] {
            let o = vec![(bad.0.to_owned(), bad.1.to_owned())];
            assert!(build(&o).is_err(), "{bad:?}");
        }
        for ok in [("vbus", "21000"), ("frames", "0"), ("chunk", "1"), ("replies", "aa")] {
            let o = vec![(ok.0.to_owned(), ok.1.to_owned())];
            assert!(build(&o).is_ok(), "{ok:?}");
        }
    }
}
