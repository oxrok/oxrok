// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! An Atten PPS3203T-3S triple power supply on the CDC-ACM shell: fixed
//! 24-byte frames at 9600 baud, each one the host sends answered by one.
//!
//! The device, and how sure this is:
//!
//! - The USB side: the sigrok wiki's `lsusb -v` shows an FT232R, `0403:6001`,
//!   "FTDI" "FT232R USB UART". This model is NOT that: `ftdi_sio` would take
//!   those ids and the tools would meet another kernel driver, so it is a
//!   CDC-ACM function with the shell's descriptors and test ids, which is
//!   an artifact of the emulation. The tools see a tty either way.
//! - The line: the wiki and the driver's comment say 9600 baud and nine data
//!   bits, the ninth unused, which 8n2 or mark parity absorb. So it sits
//!   behind a 9600/8n2 UART (`uart=` changes it). The shell lets 8n1 through
//!   too, where a real one's framing is not known.
//! - The frame: the wiki's table. Byte 0 is `aa`, byte 1 the command, then
//!   voltage in hundredths and current in thousandths for each of the three
//!   channels, big endian, at 2..14; 14 and 16 are fixed at 1; 15 the outputs
//!   as a mask; 17 the vendor software's language; 18 over-current
//!   protection; 19 the mode (independent, series, parallel); 23 the sum of
//!   the others, which the wiki says neither side checks.
//! - The commands: `aa` asks and changes nothing, as a scan must not disturb
//!   a supply. `20` sets everything in the frame, and is also the only way to
//!   get a reading once running: the wiki and the README of `pps-tools` say
//!   the device sends only when it gets a frame, and that what was set cannot
//!   be read back. No recording of a real one was found.
//! - The answer: the same layout with the measurements in the value fields.
//!   That byte 1 echoes the command is an inference: the driver's scan wants
//!   `aa aa` for its `aa`, and the wiki's table has `aa 20` for what it knows.
//!   Byte 15, 18 and 19 echo the state. The real timing is not known either.
//! - The ranges, 32 V and 3 A, with 6 V on channel 3: the wiki.
//!
//! Emulation artifacts:
//!
//! - It answers at once, where a real one takes at least the frame's 28 ms
//!   on the line, so nothing here can check a driver's pacing.
//! - It keeps a frame across pauses, however the writes cut it. How a real
//!   one resynchronises is not known.
//! - `load=` and `replies=` are stand-ins for what an output really does.
//!
//! Not modelled: what series and parallel do to the outputs (the mode is kept
//! and reported, no more), and current limiting beyond `load=` falling to
//! the limit. An `aa` frame that follows a `20` too soon is not lost.
//!
//! Options, besides the shell's ([`Faults`]):
//!
//! - `vset=A,B,C` in hundredths of a volt, `iset=A,B,C` in milliamperes,
//!   `output=MASK`, `ocp=0|1`, `mode=N`, the state at power on; `mode=` takes
//!   any byte, for a device that reports nonsense;
//! - `load=OHMS` on every output, or none;
//! - `replies=HEX[,HEX...]`, whole answers played in turn instead of the
//!   computed one, of any length up to 64: short, long or garbage ones are a
//!   device gone wrong;
//! - `answers=N`, frames answered per power on, then silence and the event
//!   point;
//! - `setters=0`, which flags any `20`, for what must change nothing;
//! - `uart=LINE`.
//!
//! The pedantic checks: the header, the command, a `20`'s fixed bytes, sum,
//! ranges, mask, protection and mode, an `aa`'s zeros, and nothing left of
//! a frame when a tool opens. An `aa` frame whose byte 23 is 0 is taken: it
//! has no sum to speak of, as the driver sends it and nothing checks it.

use crate::model::DeviceModel;
use crate::options::{parse_hex_list, parse_number};
use crate::shells::cdc::{Cdc, Faults, Identity, Line, LineEvent, Port, Serial};
use crate::shells::hidcable::Packets;
use crate::{Error, Result};
use std::fmt::Write as _;

/// The model name.
pub const NAME: &str = "atten-pps3xxx";
/// What `oxrok-emu list` says about it.
pub const ABOUT: &str = "An Atten PPS3203T-3S power supply on CDC-ACM; vset=, iset=, output=, \
                         ocp=, mode=, load=, replies=, answers=, setters=, uart=, and the \
                         shell's stall=, chunk=, in-limit=, in-stall=, out-hold=";

/// A frame's length, both ways.
const FRAME: usize = 24;
const COMMAND_ASK: u8 = 0xaa;
const COMMAND_SET: u8 = 0x20;
/// Hundredths of a volt each channel takes.
const VMAX: [u16; 3] = [3200, 3200, 600];
/// Milliamperes each channel takes.
const IMAX: u16 = 3000;
/// Answers played by `replies=` may be longer than a frame, up to this.
const LONGEST_REPLY: usize = 64;

/// A supply from the `-o` options.
///
/// # Errors
///
/// On an unknown option or a value out of range.
pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>> {
    Ok(Box::new(function(options)?))
}

fn three(key: &str, value: &str, limits: [u16; 3]) -> Result<[u16; 3]> {
    let parts: Vec<&str> = value.split(',').collect();
    if parts.len() != 3 {
        return Err(Error(format!("{key}={value}: three numbers, one per channel")));
    }
    let mut out = [0; 3];
    for (i, part) in parts.iter().enumerate() {
        let n = parse_number(key, part, 0..=u32::from(limits[i]))?;
        out[i] = u16::try_from(n).expect("within a u16 limit");
    }
    Ok(out)
}

fn function(options: &[(String, String)]) -> Result<Cdc<Supply>> {
    let mut faults = Faults::default();
    let mut uart = Line::parse("9600/8n2")?;
    let mut config = Config::default();
    for (k, v) in options {
        if faults.option(k, v)? {
            continue;
        }
        match k.as_str() {
            "vset" => config.power_on.vset = three(k, v, VMAX)?,
            "iset" => config.power_on.iset = three(k, v, [IMAX; 3])?,
            "output" => {
                config.power_on.output = u8::try_from(parse_number(k, v, 0..=7)?).expect("mask");
            }
            "ocp" => config.power_on.ocp = parse_number(k, v, 0..=1)? == 1,
            "mode" => {
                config.power_on.mode = u8::try_from(parse_number(k, v, 0..=255)?).expect("byte");
            }
            "load" => config.load = Some(parse_number(k, v, 1..=1_000_000)?),
            "replies" => {
                config.replies = Some(Packets::new(parse_hex_list(k, v, 1, LONGEST_REPLY)?));
            }
            "answers" => config.answers = Some(parse_number(k, v, 0..=100_000)?),
            "setters" => config.setters = parse_number(k, v, 0..=1)? == 1,
            "uart" => uart = Line::parse(v)?,
            other => return Err(Error(format!("{NAME} has no option {other}"))),
        }
    }
    // Test ids: see the module's note on the real FT232R.
    let id = Identity::new(0x1209, 0x0001, "oxrok-emu", "Atten PPS3203T-3S stand-in");
    Ok(Cdc::new(NAME, &id, faults, Supply::new(config)).behind_uart(uart))
}

#[derive(Clone, Copy, Debug)]
struct Panel {
    vset: [u16; 3],
    iset: [u16; 3],
    output: u8,
    ocp: bool,
    mode: u8,
}

#[derive(Clone, Debug)]
struct Config {
    power_on: Panel,
    load: Option<u32>,
    replies: Option<Packets>,
    answers: Option<u32>,
    setters: bool,
}

impl Default for Config {
    fn default() -> Config {
        Config {
            power_on: Panel {
                vset: [1234, 500, 330],
                iset: [1000, 500, 250],
                output: 0,
                ocp: false,
                mode: 0,
            },
            load: None,
            replies: None,
            answers: None,
            setters: true,
        }
    }
}

struct Supply {
    config: Config,
    now: Panel,
    replies: Option<Packets>,
    /// What has come of a frame so far, shorter than a frame.
    pending: Vec<u8>,
    answered: u32,
}

impl Supply {
    fn new(config: Config) -> Supply {
        Supply {
            now: config.power_on,
            replies: config.replies.clone(),
            config,
            pending: Vec::new(),
            answered: 0,
        }
    }

    /// One output's volts in hundredths and amperes in thousandths.
    fn operating_point(&self, channel: usize) -> (u16, u16) {
        if self.now.output & (1 << channel) == 0 {
            return (0, 0);
        }
        let (vset, iset) = (self.now.vset[channel], self.now.iset[channel]);
        let Some(load) = self.config.load else { return (vset, 0) };
        let wanted = u64::from(vset) * 10 / u64::from(load);
        if wanted <= u64::from(iset) {
            (vset, u16::try_from(wanted).expect("below iset"))
        } else {
            let volts = u64::from(iset) * u64::from(load) / 10;
            (u16::try_from(volts).unwrap_or(u16::MAX).min(vset), iset)
        }
    }

    fn answer(&mut self, command: u8) -> Vec<u8> {
        if let Some(list) = &mut self.replies {
            return list.take();
        }
        let mut frame = vec![0; FRAME];
        frame[0] = COMMAND_ASK;
        frame[1] = command;
        for channel in 0..3 {
            let (volts, amps) = self.operating_point(channel);
            frame[2 + channel * 4..4 + channel * 4].copy_from_slice(&volts.to_be_bytes());
            frame[4 + channel * 4..6 + channel * 4].copy_from_slice(&amps.to_be_bytes());
        }
        frame[14] = 1;
        frame[15] = self.now.output;
        frame[16] = 1;
        frame[18] = u8::from(self.now.ocp);
        frame[19] = self.now.mode;
        frame[23] = sum(&frame);
        frame
    }

    /// A `20`'s values, when all of them hold.
    fn set(&mut self, f: &[u8], port: &mut Port<'_>) {
        if !self.config.setters {
            port.violation(format!("{}: a setting no one asked for", show(f)));
        }
        let mut good = true;
        let mut bad = |why: String| {
            port.violation(format!("{}: {why}", show(f)));
            good = false;
        };
        if f[14] != 1 || f[16] != 1 {
            bad("bytes 14 and 16 must be 1".to_owned());
        }
        if f[17] != 0 || f[20..23] != [0, 0, 0] {
            bad("bytes 17 and 20 to 22 must be 0".to_owned());
        }
        if f[15] > 7 {
            bad("outputs beyond the third".to_owned());
        }
        if f[18] > 1 {
            bad("protection is 0 or 1".to_owned());
        }
        if f[19] > 2 {
            bad("no such mode".to_owned());
        }
        if f[23] != sum(f) {
            bad("wrong sum".to_owned());
        }
        let mut vset = [0; 3];
        let mut iset = [0; 3];
        for channel in 0..3 {
            let at = 2 + channel * 4;
            vset[channel] = u16::from_be_bytes([f[at], f[at + 1]]);
            iset[channel] = u16::from_be_bytes([f[at + 2], f[at + 3]]);
            if vset[channel] > VMAX[channel] || iset[channel] > IMAX {
                bad(format!("channel {} out of range", channel + 1));
            }
        }
        if good {
            self.now = Panel { vset, iset, output: f[15], ocp: f[18] == 1, mode: f[19] };
        }
    }

    fn frame(&mut self, f: &[u8], port: &mut Port<'_>) {
        if f[0] != COMMAND_ASK {
            port.violation(format!("{}: no header", show(f)));
            return;
        }
        match f[1] {
            COMMAND_ASK if f[2..].iter().all(|b| *b == 0) => {}
            COMMAND_ASK => {
                port.violation(format!("{}: an ask with values", show(f)));
                return;
            }
            COMMAND_SET => self.set(f, port),
            _ => {
                port.violation(format!("{}: no such command", show(f)));
                return;
            }
        }
        if self.config.answers.is_some_and(|n| self.answered >= n) {
            port.mark_event_point();
            return;
        }
        self.answered += 1;
        let answer = self.answer(f[1]);
        port.send(&answer);
    }
}

fn sum(frame: &[u8]) -> u8 {
    frame[..FRAME - 1].iter().fold(0, |s: u8, b| s.wrapping_add(*b))
}

fn show(bytes: &[u8]) -> String {
    bytes.iter().fold(String::new(), |mut s, b| {
        let _ = write!(s, "{b:02x}");
        s
    })
}

impl Serial for Supply {
    fn received(&mut self, data: &[u8], port: &mut Port<'_>) {
        // Bounded: what is left of `pending` after the loop is under a frame.
        for chunk in data.chunks(FRAME) {
            self.pending.extend_from_slice(chunk);
            while self.pending.len() >= FRAME {
                let frame: Vec<u8> = self.pending.drain(..FRAME).collect();
                self.frame(&frame, port);
            }
        }
    }

    fn changed(&mut self, event: LineEvent, port: &mut Port<'_>) {
        if event.opened() && !self.pending.is_empty() {
            port.violation(format!("{}: a frame left unfinished", show(&self.pending)));
            self.pending.clear();
        }
    }

    fn reset(&mut self) {
        *self = Supply::new(self.config.clone());
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::shells::cdc::host::Host;

    fn supply(options: &[(&str, &str)]) -> Host<Supply> {
        let o: Vec<(String, String)> =
            options.iter().map(|(k, v)| ((*k).to_owned(), (*v).to_owned())).collect();
        Host::open(function(&o).unwrap(), Line::parse("9600/8n2").unwrap(), true, true)
    }

    fn ask() -> Vec<u8> {
        let mut f = vec![0; FRAME];
        f[0] = 0xaa;
        f[1] = 0xaa;
        f
    }

    /// A good `20`, with channel 1's volts and amperes and the mask.
    fn setting(volts: u16, amps: u16, output: u8) -> Vec<u8> {
        let mut f = vec![0; FRAME];
        f[0] = 0xaa;
        f[1] = 0x20;
        f[2..4].copy_from_slice(&volts.to_be_bytes());
        f[4..6].copy_from_slice(&amps.to_be_bytes());
        f[14] = 1;
        f[15] = output;
        f[16] = 1;
        f[23] = sum(&f);
        f
    }

    fn violations(h: &mut Host<Supply>) -> usize {
        h.v.drain().len()
    }

    #[test]
    fn an_ask_answers_and_changes_nothing() {
        let mut h = supply(&[("output", "5"), ("ocp", "1"), ("mode", "2")]);
        h.write(&ask());
        let a = h.read();
        assert_eq!(a.len(), FRAME);
        assert_eq!(a[..2], [0xaa, 0xaa]);
        assert_eq!(a[2..4], 1234u16.to_be_bytes(), "CH1's output is on");
        assert_eq!(a[6..8], [0, 0], "CH2's is off");
        assert_eq!(a[10..12], 330u16.to_be_bytes());
        assert_eq!((a[14], a[15], a[16]), (1, 5, 1));
        assert_eq!((a[18], a[19]), (1, 2));
        assert_eq!(a[23], sum(&a));
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn a_setting_takes_effect_and_answers() {
        let mut h = supply(&[("load", "10")]);
        h.write(&setting(500, 2000, 1));
        let a = h.read();
        assert_eq!(a[..2], [0xaa, 0x20]);
        assert_eq!(a[2..4], 500u16.to_be_bytes());
        assert_eq!(a[4..6], 500u16.to_be_bytes(), "5 V on 10 ohms");
        assert_eq!(a[15], 1);
        assert_eq!(a[6..8], [0, 0], "and the rest went to 0 and off");
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn a_setting_zeroes_what_it_does_not_name() {
        let mut h = supply(&[("output", "7")]);
        h.write(&setting(0, 0, 0));
        h.read();
        assert_eq!(h.cdc.model.now.vset, [0, 0, 0]);
        assert_eq!(h.cdc.model.now.output, 0);
    }

    #[test]
    fn a_load_too_heavy_falls_to_the_limit() {
        let mut h = supply(&[("load", "1")]);
        h.write(&setting(1200, 2000, 1));
        let a = h.read();
        assert_eq!(a[2..4], 200u16.to_be_bytes());
        assert_eq!(a[4..6], 2000u16.to_be_bytes());
    }

    #[test]
    fn a_frame_may_come_in_pieces_or_together() {
        let mut h = supply(&[]);
        let f = ask();
        h.write(&f[..5]);
        assert_eq!(h.read(), b"");
        h.write(&f[5..]);
        assert_eq!(h.read().len(), FRAME);
        let two: Vec<u8> = f.iter().chain(f.iter()).copied().collect();
        h.write(&two);
        assert_eq!(h.read().len(), 2 * FRAME);
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn a_long_write_keeps_the_buffer_short() {
        let mut h = supply(&[]);
        h.write(&[0x55; 4096]);
        assert!(h.cdc.model.pending.len() < FRAME);
        assert_eq!(violations(&mut h), 4096 / FRAME);
    }

    #[test]
    fn each_check_fires_and_changes_nothing() {
        let change = |at: usize, value: u8| {
            let mut f = setting(500, 1000, 1);
            f[at] = value;
            f[23] = sum(&f);
            f
        };
        let mut wrong_sum = setting(500, 1000, 1);
        wrong_sum[23] ^= 1;
        let mut ask_with_values = ask();
        ask_with_values[5] = 1;
        let mut no_such = ask();
        no_such[1] = 0x21;
        let cases = [
            change(14, 0),
            change(16, 2),
            change(17, 1),
            change(20, 1),
            change(22, 1),
            change(15, 8),
            change(18, 2),
            change(19, 3),
            setting(3201, 0, 0),
            setting(0, 3001, 0),
            wrong_sum,
        ];
        for bad in &cases {
            let mut h = supply(&[]);
            h.write(bad);
            assert_eq!(violations(&mut h), 1, "{}", show(bad));
            assert_eq!(h.cdc.model.now.vset[0], 1234, "{}", show(bad));
        }
        let mut channel_3 = setting(0, 0, 0);
        channel_3[10..12].copy_from_slice(&601u16.to_be_bytes());
        channel_3[23] = sum(&channel_3);
        let mut no_header = ask();
        no_header[0] = 0x55;
        for bad in [channel_3, ask_with_values, no_such, no_header] {
            let mut h = supply(&[]);
            h.write(&bad);
            assert_eq!(violations(&mut h), 1, "{}", show(&bad));
        }
    }

    #[test]
    fn the_wrong_frames_get_no_answer_but_a_bad_setting_still_does() {
        let mut no_such = ask();
        no_such[1] = 0x21;
        let mut h = supply(&[]);
        h.write(&no_such);
        assert_eq!(h.read(), b"");
        h.write(&setting(3201, 0, 0));
        assert_eq!(h.read().len(), FRAME, "a real one answers every frame it takes");
    }

    #[test]
    fn setters_0_flags_a_setting_and_not_an_ask() {
        let mut h = supply(&[("setters", "0")]);
        h.write(&ask());
        assert_eq!(violations(&mut h), 0);
        h.write(&setting(0, 0, 0));
        assert_eq!(violations(&mut h), 1);
    }

    #[test]
    fn a_frame_left_unfinished_shows_at_the_next_open() {
        let mut h = supply(&[]);
        h.write(&[0xaa, 0xaa]);
        h.set_lines(false, false);
        assert_eq!(violations(&mut h), 0);
        h.set_lines(true, true);
        assert_eq!(violations(&mut h), 1);
        assert_eq!(h.cdc.model.pending, b"");
    }

    #[test]
    fn answers_runs_out_and_marks_the_point() {
        let mut h = supply(&[("answers", "1")]);
        h.write(&ask());
        assert_eq!(h.read().len(), FRAME);
        assert!(!h.cdc.take_event_point());
        h.write(&ask());
        assert_eq!(h.read(), b"");
        assert!(h.cdc.take_event_point());
        h.write(&setting(100, 100, 1));
        assert_eq!(h.cdc.model.now.vset[0], 100, "a silent one still takes a setting");
    }

    #[test]
    fn replies_play_in_turn() {
        let mut h = supply(&[("replies", "aabb,cc")]);
        for want in [&[0xaa, 0xbb][..], &[0xcc], &[0xaa, 0xbb]] {
            h.write(&ask());
            assert_eq!(h.read(), want);
        }
    }

    #[test]
    fn it_listens_at_its_line_only() {
        let mut h = supply(&[]);
        h.set_line(Line::parse("19200/8n2").unwrap());
        h.write(&ask());
        assert_eq!(h.read(), b"");
        let mut h = supply(&[("uart", "19200/8n2")]);
        h.set_line(Line::parse("19200/8n2").unwrap());
        h.write(&ask());
        assert_eq!(h.read().len(), FRAME);
    }

    #[test]
    fn a_reset_goes_back_to_power_on() {
        let mut h = supply(&[("answers", "1"), ("replies", "aa,bb")]);
        h.write(&setting(100, 100, 1));
        assert_eq!(h.read(), [0xaa]);
        h.cdc.model.reset();
        assert_eq!(h.cdc.model.now.vset[0], 1234);
        h.write(&ask());
        assert_eq!(h.read(), [0xaa], "the list and the count start over");
    }

    #[test]
    fn options_parse() {
        for bad in [
            ("vset", "1,2"),
            ("vset", "3201,0,0"),
            ("vset", "0,0,601"),
            ("iset", "3001,0,0"),
            ("output", "8"),
            ("ocp", "2"),
            ("mode", "256"),
            ("load", "0"),
            ("replies", "abc"),
            ("uart", "9600"),
            ("nothing", "1"),
        ] {
            let o = vec![(bad.0.to_owned(), bad.1.to_owned())];
            assert!(build(&o).is_err(), "{bad:?}");
        }
        for ok in [("vset", "3200,3200,600"), ("mode", "255"), ("chunk", "1"), ("replies", "aa")] {
            let o = vec![(ok.0.to_owned(), ok.1.to_owned())];
            assert!(build(&o).is_ok(), "{ok:?}");
        }
    }
}
