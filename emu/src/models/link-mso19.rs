// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! A Link Instruments MSO-19 on the CDC-ACM shell: one analog and eight logic
//! inputs, 1024 samples of memory, registers written in framed words.
//!
//! The device, and how sure this is:
//!
//! - Ids 3195:f190: libsigrok's driver, and a second program written against
//!   real units (tkrmnz/mso19fcgi, `main.c`, which asks `lsusb -d 3195:f190`).
//! - The frame, `@LDS~` then big endian words then `~`, and the word's layout:
//!   both programs build them alike. Each word has the register in bits 8 to 11
//!   and the value's bits around them, with bit 6 the inverse of value bit 5 and
//!   bit 14 the inverse of value bit 7. So no word holds `~`, XON or XOFF.
//! - The registers and their banks: libsigrok's driver. mso19fcgi uses the same
//!   addresses for what it shares. Neither is a data sheet, and no firmware or
//!   HDL source was found.
//! - The serial number holds the calibration: 5 digits of volts per bit, 3 of
//!   the DAC's centre, 3 of its scale, model and revision digits, 6 of the unit's
//!   number. mso19fcgi reads 11 more digits after those, the same three values
//!   for 200 MSa/s (`GetVbit` under 10 ns a sample), and takes a 30 digit number,
//!   so that is what this has. libsigrok reads only the first 19.
//! - A status read answers one byte. mso19fcgi masks it with 0x1f and takes 22,
//!   0x16, as "waiting for readback"; libsigrok's names are action 6 and the
//!   armed bit, with 0x20 as "OK". What bit 5 does here (always set) is a guess.
//!   So are the idle value (0x21) and how many reads each stage lasts.
//! - The dump: 3072 bytes, three per sample, the analog value in the 6 low bits
//!   of the first byte and the 4 low bits of the second, the logic byte in bits 4
//!   and 5 of the second and the 6 low bits of the third. Both programs decode
//!   it alike. Neither says what the two high bits hold.
//!
//! What the two high bits of a dump's byte hold is a guess, which matters:
//! libsigrok opens the line with XON/XOFF, and the tty takes those bytes out of
//! the input, so a dump holding one comes up short. mso19fcgi clears `IXON` and
//! takes "at least 3070 bytes". libsigrok has had `flow=2` since 2013, and its
//! 2020 commit 6e183da9 says it gets "basic analog and digital data working", which
//! suggests it holds on a unit, though no message mentions XON/XOFF. So a byte here
//! has bit 6 the inverse of its bit 5, as in a word, which holds neither. `raw=1`
//! leaves them 0, and the dump then holds both.
//!
//! Emulation artifacts:
//!
//! - It has no inputs, so no setting changes what it samples and a trigger is a
//!   count of status reads: `pre=`, `wait=` and `post=` say how many reads each
//!   stage answers. The samples are a stand-in, each analog value and each logic
//!   value once or more, the same every run.
//! - The strings "Link Instruments" and "MSO-19" are guesses, from the ids.
//! - The USB side is the shell's CDC-ACM. It takes the line settings and does
//!   nothing with them, as a native USB port would; whether the real unit does is
//!   not known.
//! - It answers at once, unless `late-at=` or `pause=` hold something back.
//!
//! Options, besides the shell's ([`Faults`]):
//!
//! - `vid=`, `pid=`, `manufacturer=`, `product=`, `serial=`, the descriptors'; an
//!   empty string leaves the kernel with none;
//! - `pre=N`, `wait=N|never`, `post=N`, the status reads each stage answers after
//!   an arm, 1 each by default, all 0 for a capture done before the first read;
//! - `status-at=N:HEX`, the Nth status read since power on answers that byte;
//! - `mute-at=N`, the Nth is not answered and the event point follows;
//! - `late-at=N:T`, the Nth is answered after T offers of a read;
//! - `mark-at=N`, the event point at the first offer after the Nth;
//! - `dump=N`, `extra=N`, a dump of N bytes, then the event point, or N bytes over;
//! - `pause=N:T`, a dump waits T offers after its first N bytes;
//! - `raw=1`, a dump's high bits left 0.
//!
//! The pedantic checks: bytes outside a frame, a word with its inverse bits
//! wrong, an odd or empty frame, a frame cut off at an open, a register no bank
//! has or that no C path writes (bank 1, 3), a read with a value, a status or dump
//! read outside bank 0, a dump before the data is ready, an arm with the ADC off,
//! and the bits that have no name in `REG_CTL1`, `REG_CTL2` and the DAC's word.
//! Values are not held to libsigrok's tables: mso19fcgi writes dividers they lack.

use crate::model::DeviceModel;
use crate::options::parse_number;
use crate::shells::cdc::{Cdc, Faults, Identity, LineEvent, Port, Serial};
use crate::{Error, Result};

/// The model name.
pub const NAME: &str = "link-mso19";
/// What `oxrok-emu list` says about it.
pub const ABOUT: &str = "A Link Instruments MSO-19 on CDC-ACM; vid=, pid=, manufacturer=, \
                         product=, serial=, pre=, wait=, post=, status-at=, mute-at=, late-at=, \
                         mark-at=, dump=, extra=, pause=, raw=, and the shell's stall=, chunk=, \
                         in-limit=, in-stall=, out-hold=";

/// Samples in the memory.
const SAMPLES: usize = 1024;
/// Bytes in a dump.
const DUMP: usize = SAMPLES * 3;
/// A frame's start.
const HEAD: &[u8] = b"@LDS~";
/// A frame's end, which no word holds.
const FOOT: u8 = b'~';
/// More than the longest frame the host has a reason to send.
const LONGEST: usize = 128;
/// A serial number as mso19fcgi reads one, with libsigrok's defaults in it.
const SERIAL: &str = "428743435001200012342874343500";

const REG_BUFFER: u8 = 1;
const REG_STATUS: u8 = 2;
const REG_DAC_MSB: u8 = 12;
const REG_DAC_LSB: u8 = 13;
const REG_CTL1: u8 = 14;
const REG_CTL2: u8 = 15;
/// The registers of bank 2 stop here: 4 words, 4 masks and the SPI mode.
const BANK2_LAST: u8 = 8;

const CTL1_RESET_FSM: u8 = 1 << 0;
const CTL1_ARM: u8 = 1 << 1;
const CTL1_RESERVED: u8 = 1 << 2;
const CTL1_FORCE: u8 = 1 << 3;
const CTL1_ADC_ENABLE: u8 = 1 << 4;
const CTL1_LOAD_DAC: u8 = 1 << 5;
const CTL1_RESET_ADC: u8 = 1 << 6;

const CTL2_BANK: u8 = 0x3;
const CTL2_SLOW: u8 = 1 << 5;

const STATUS_IDLE: u8 = 0x21;
const STATUS_ARMED: u8 = 0x30;
const STATUS_PRE_FILL: u8 = 3;
const STATUS_WAIT: u8 = 4;
const STATUS_POST_FILL: u8 = 5;
const STATUS_READY: u8 = 6;

/// A device from the `-o` options.
///
/// # Errors
///
/// On an unknown option or a value out of range.
pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>> {
    Ok(Box::new(function(options)?))
}

fn number(key: &str, value: &str) -> Result<u32> {
    parse_number(key, value, 0..=1_000_000)
}

fn word16(key: &str, value: &str) -> Result<u16> {
    Ok(u16::try_from(parse_number(key, value, 0..=0xffff)?).unwrap_or(u16::MAX))
}

fn dump_bytes(key: &str, value: &str) -> Result<usize> {
    let most = u32::try_from(DUMP).unwrap_or(u32::MAX);
    Ok(usize::try_from(parse_number(key, value, 0..=most)?).unwrap_or(DUMP))
}

/// `N:VALUE`, a count and what comes after the colon.
fn at(key: &str, value: &str) -> Result<(u32, String)> {
    let (n, rest) = value
        .split_once(':')
        .ok_or_else(|| Error::Invalid(format!("{key}={value}: N:VALUE")))?;
    Ok((number(key, n)?, rest.to_owned()))
}

fn function(options: &[(String, String)]) -> Result<Cdc<Mso19>> {
    let mut faults = Faults::default();
    let mut id = Identity::new(0x3195, 0xf190, "Link Instruments", "MSO-19");
    SERIAL.clone_into(&mut id.serial);
    let mut config = Config::default();
    for (k, v) in options {
        if faults.option(k, v)? {
            continue;
        }
        match k.as_str() {
            "vid" => id.vendor = word16(k, v)?,
            "pid" => id.product = word16(k, v)?,
            "manufacturer" => v.clone_into(&mut id.manufacturer),
            "product" => v.clone_into(&mut id.name),
            "serial" => v.clone_into(&mut id.serial),
            "pre" => config.pre = number(k, v)?,
            "wait" => config.wait = if v == "never" { None } else { Some(number(k, v)?) },
            "post" => config.post = number(k, v)?,
            "status-at" => {
                let (n, byte) = at(k, v)?;
                let byte = u8::from_str_radix(&byte, 16)
                    .map_err(|_| Error::Invalid(format!("{k}={v}: N:HEX")))?;
                config.status_at = Some((n, byte));
            }
            "mute-at" => config.mute_at = Some(number(k, v)?),
            "late-at" => {
                let (n, offers) = at(k, v)?;
                config.late_at = Some((n, number(k, &offers)?));
            }
            "mark-at" => config.mark_at = Some(number(k, v)?),
            "dump" => {
                config.dump = dump_bytes(k, v)?;
                config.silent_after_dump = true;
            }
            "extra" => config.extra = dump_bytes(k, v)?,
            "pause" => {
                let (n, offers) = at(k, v)?;
                config.pause = Some((dump_bytes(k, &n.to_string())?, number(k, &offers)?));
            }
            "raw" => config.raw = parse_number(k, v, 0..=1)? == 1,
            _ => return Err(Error::Invalid(format!("{NAME} has no option {k}"))),
        }
    }
    let samples = samples(config.raw);
    Ok(Cdc::new(NAME, &id, faults, Mso19::new(config, samples)))
}

/// One sample's analog and logic value.
fn sample(i: usize) -> (u16, u8) {
    let analog = u16::try_from(i.wrapping_mul(3) & 0x3ff).unwrap_or(0);
    let logic = u8::try_from((i ^ (i >> 3)) & 0xff).unwrap_or(0);
    (analog, logic)
}

/// A dump's byte, its 6 low bits and what the high ones hold.
fn dump_byte(value: u8, raw: bool) -> u8 {
    let low = value & 0x3f;
    if raw || low & 0x20 != 0 { low } else { low | 0x40 }
}

/// The memory as a dump sends it, made when the model is built.
fn samples(raw: bool) -> Vec<u8> {
    let mut dump = Vec::with_capacity(DUMP);
    for i in 0..SAMPLES {
        let (analog, logic) = sample(i);
        let [low, high] = analog.to_le_bytes();
        dump.push(dump_byte(low & 0x3f, raw));
        dump.push(dump_byte(((low >> 6) | (high << 2)) & 0x0f | ((logic & 3) << 4), raw));
        dump.push(dump_byte(logic >> 2, raw));
    }
    dump
}

#[derive(Clone)]
struct Config {
    pre: u32,
    wait: Option<u32>,
    post: u32,
    status_at: Option<(u32, u8)>,
    mute_at: Option<u32>,
    late_at: Option<(u32, u32)>,
    mark_at: Option<u32>,
    dump: usize,
    silent_after_dump: bool,
    extra: usize,
    pause: Option<(usize, u32)>,
    raw: bool,
}

impl Default for Config {
    fn default() -> Config {
        Config {
            pre: 1,
            wait: Some(1),
            post: 1,
            status_at: None,
            mute_at: None,
            late_at: None,
            mark_at: None,
            dump: DUMP,
            silent_after_dump: false,
            extra: 0,
            pause: None,
            raw: false,
        }
    }
}

/// Where a capture is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Stage {
    Idle,
    PreFill(u32),
    Wait(Option<u32>),
    PostFill(u32),
    Ready,
}

/// What is held back from the host, in offers of a read.
#[derive(Clone, Debug, Default)]
struct Held {
    offers: u32,
    bytes: Vec<u8>,
}

/// The state since power on.
#[derive(Clone)]
struct Now {
    stage: Stage,
    ctl1: u8,
    ctl2: u8,
    dac: [u8; 2],
    status_reads: u32,
    /// The event point is due at the next offer.
    mark: bool,
    held: Option<Held>,
}

impl Now {
    fn power_on() -> Now {
        Now {
            stage: Stage::Idle,
            ctl1: 0,
            ctl2: 0,
            dac: [0; 2],
            status_reads: 0,
            mark: false,
            held: None,
        }
    }
}

/// The instrument behind the shell.
pub struct Mso19 {
    config: Config,
    dump: Vec<u8>,
    pending: Vec<u8>,
    now: Now,
}

impl Mso19 {
    fn new(config: Config, dump: Vec<u8>) -> Mso19 {
        Mso19 { config, dump, pending: Vec::new(), now: Now::power_on() }
    }

    fn bank(&self) -> u8 {
        self.now.ctl2 & CTL2_BANK
    }

    /// The first stage a capture with these counts answers.
    fn armed_stage(&self) -> Stage {
        self.settle(Stage::PreFill(self.config.pre))
    }

    /// `stage`, or the first one after it with reads left.
    fn settle(&self, stage: Stage) -> Stage {
        match stage {
            Stage::PreFill(0) => self.settle(Stage::Wait(self.config.wait)),
            Stage::Wait(Some(0)) => self.settle(Stage::PostFill(self.config.post)),
            Stage::PostFill(0) => Stage::Ready,
            other => other,
        }
    }

    /// The byte for a status read, and one read off the stage's count.
    fn status(&mut self) -> u8 {
        let (byte, next) = match self.now.stage {
            Stage::Idle => (STATUS_IDLE, Stage::Idle),
            Stage::PreFill(n) => (STATUS_ARMED | STATUS_PRE_FILL, Stage::PreFill(n - 1)),
            Stage::Wait(n) => (STATUS_ARMED | STATUS_WAIT, Stage::Wait(n.map(|n| n - 1))),
            Stage::PostFill(n) => (STATUS_ARMED | STATUS_POST_FILL, Stage::PostFill(n - 1)),
            Stage::Ready => (STATUS_ARMED | STATUS_READY, Stage::Ready),
        };
        self.now.stage = self.settle(next);
        byte
    }

    fn read_status(&mut self, port: &mut Port<'_>) {
        self.now.status_reads += 1;
        let n = self.now.status_reads;
        if self.config.mark_at == Some(n) {
            self.now.mark = true;
        }
        let byte = self.status();
        if self.config.mute_at == Some(n) {
            self.now.mark = true;
            return;
        }
        let byte = match self.config.status_at {
            Some((at, forced)) if at == n => forced,
            _ => byte,
        };
        match self.config.late_at {
            Some((at, offers)) if at == n => {
                self.now.held = Some(Held { offers, bytes: vec![byte] });
            }
            _ => port.send(&[byte]),
        }
    }

    fn read_buffer(&mut self, port: &mut Port<'_>) {
        if self.now.stage != Stage::Ready {
            port.violation(format!("a dump asked for at {:?}", self.now.stage));
            return;
        }
        let mut bytes = self.dump[..self.config.dump].to_vec();
        bytes.resize(self.config.dump + self.config.extra, 0x40);
        if self.config.silent_after_dump {
            self.now.mark = true;
        }
        match self.config.pause {
            Some((first, offers)) if first < bytes.len() => {
                let rest = bytes.split_off(first);
                port.send(&bytes);
                self.now.held = Some(Held { offers, bytes: rest });
            }
            _ => port.send(&bytes),
        }
    }

    fn write_ctl1(&mut self, value: u8, port: &mut Port<'_>) {
        if value & CTL1_RESERVED != 0 {
            port.violation(format!("REG_CTL1 {value:#04x}: bit 2 has no name"));
        }
        self.now.ctl1 = value;
        if value & (CTL1_RESET_ADC | CTL1_RESET_FSM) != 0 {
            self.now.stage = Stage::Idle;
        }
        if value & CTL1_ARM != 0 {
            if value & CTL1_ADC_ENABLE == 0 {
                port.violation(format!("REG_CTL1 {value:#04x}: an arm with the ADC off"));
            }
            self.now.stage = self.armed_stage();
        }
        if value & CTL1_FORCE != 0 && matches!(self.now.stage, Stage::Wait(_)) {
            self.now.stage = self.settle(Stage::PostFill(self.config.post));
        }
        if value & CTL1_LOAD_DAC != 0 && self.now.dac[0] & 0x70 != 0 {
            let [msb, lsb] = self.now.dac;
            port.violation(format!("a DAC word {msb:02x}{lsb:02x} with bits 12 to 14 set"));
        }
    }

    /// One word's register and value, or what is wrong with it.
    fn write(&mut self, register: u8, value: u8, port: &mut Port<'_>) {
        if register == REG_CTL2 {
            if value & !(CTL2_BANK | CTL2_SLOW) != 0 {
                port.violation(format!("REG_CTL2 {value:#04x}: a bit with no name"));
            }
            self.now.ctl2 = value;
            if !matches!(self.bank(), 0 | 2) {
                port.violation(format!("REG_CTL2 {value:#04x}: a bank no C path selects"));
            }
            return;
        }
        match self.bank() {
            0 => match register {
                REG_BUFFER | REG_STATUS if value != 0 => {
                    port.violation(format!("a read of register {register} with value {value}"));
                }
                REG_BUFFER => self.read_buffer(port),
                REG_STATUS => self.read_status(port),
                REG_DAC_MSB => self.now.dac[0] = value,
                REG_DAC_LSB => self.now.dac[1] = value,
                REG_CTL1 => self.write_ctl1(value, port),
                3..=11 => {}
                _ => port.violation(format!("register {register} in bank 0")),
            },
            2 if register <= BANK2_LAST => {}
            bank => port.violation(format!("register {register} in bank {bank}")),
        }
    }

    /// A frame's words.
    fn frame(&mut self, words: &[u8], port: &mut Port<'_>) {
        if words.is_empty() || !words.len().is_multiple_of(2) {
            port.violation(format!("a frame of {} bytes", words.len()));
            return;
        }
        for word in words.as_chunks::<2>().0 {
            let (high, low) = (word[0], word[1]);
            let value = (low & 0x3f) | ((high >> 4) & 0x3) << 6;
            let wrong_low = (low & 0x40 != 0) == (value & 0x20 != 0);
            let wrong_high = (high & 0x40 != 0) == (value & 0x80 != 0);
            if wrong_low || wrong_high || (low | high) & 0x80 != 0 {
                port.violation(format!("a word {high:02x}{low:02x} with its fixed bits wrong"));
                continue;
            }
            self.write(high & 0xf, value, port);
        }
    }

    /// Takes whole frames from what has come.
    fn frames(&mut self, port: &mut Port<'_>) {
        loop {
            let head = self.pending.windows(HEAD.len()).position(|w| w == HEAD);
            if let Some(n) = head
                && n > 0
            {
                port.violation(format!("{n} bytes outside a frame"));
                self.pending.drain(..n);
            }
            if head.is_none() {
                // What may still become a head stays.
                let keep = (1..HEAD.len())
                    .rev()
                    .find(|n| self.pending.ends_with(&HEAD[..*n]))
                    .unwrap_or(0);
                let stray = self.pending.len() - keep;
                if stray > 0 {
                    port.violation(format!("{stray} bytes outside a frame"));
                    self.pending.drain(..stray);
                }
                return;
            }
            let Some(end) = self.pending[HEAD.len()..].iter().position(|b| *b == FOOT) else {
                if self.pending.len() > LONGEST {
                    port.violation(format!("a frame over {LONGEST} bytes"));
                    self.pending.clear();
                }
                return;
            };
            let words: Vec<u8> = self.pending[HEAD.len()..HEAD.len() + end].to_vec();
            self.pending.drain(..=HEAD.len() + end);
            self.frame(&words, port);
        }
    }
}

impl Serial for Mso19 {
    fn received(&mut self, data: &[u8], port: &mut Port<'_>) {
        self.pending.extend_from_slice(data);
        self.frames(port);
    }

    fn changed(&mut self, event: LineEvent, port: &mut Port<'_>) {
        if event.opened() && !self.pending.is_empty() {
            port.violation(format!("{} bytes of a frame left at an open", self.pending.len()));
            self.pending.clear();
        }
    }

    fn tick(&mut self, port: &mut Port<'_>) {
        if self.now.mark {
            self.now.mark = false;
            port.mark();
        }
        if let Some(held) = &mut self.now.held {
            if held.offers > 0 {
                held.offers -= 1;
                return;
            }
            port.send(&held.bytes);
            self.now.held = None;
        }
    }

    fn reset(&mut self) {
        self.pending.clear();
        self.now = Now::power_on();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::shells::cdc::Line;
    use crate::shells::cdc::host::Host;

    const WAIT: &[u8] = b"@LDS~";

    fn device(options: &[(&str, &str)]) -> Host<Mso19> {
        let o: Vec<_> = options.iter().map(|(k, v)| ((*k).to_owned(), (*v).to_owned())).collect();
        Host::open(function(&o).unwrap(), Line::parse("460800/8n1").unwrap(), true, true)
    }

    fn violations(h: &mut Host<Mso19>) -> usize {
        h.v.drain().len()
    }

    /// `mso_trans`, written from the layout above.
    fn word(register: u8, value: u8) -> [u8; 2] {
        let high = register & 0xf
            | ((value >> 6) & 3) << 4
            | if value & 0x80 == 0 { 0x40 } else { 0 };
        let low = value & 0x3f | if value & 0x20 == 0 { 0x40 } else { 0 };
        [high, low]
    }

    fn frame(words: &[[u8; 2]]) -> Vec<u8> {
        let mut out = WAIT.to_vec();
        for w in words {
            out.extend_from_slice(w);
        }
        out.push(b'~');
        out
    }

    fn status(h: &mut Host<Mso19>) -> Vec<u8> {
        h.write(&frame(&[word(REG_STATUS, 0)]));
        h.read()
    }

    fn arm(h: &mut Host<Mso19>) {
        h.write(&frame(&[
            word(REG_CTL1, 0x11),
            word(REG_CTL1, 0x12),
            word(REG_CTL1, 0x10),
        ]));
    }

    /// Everything a dump sends, over as many reads as it takes.
    fn dump(h: &mut Host<Mso19>) -> Vec<u8> {
        h.write(&frame(&[word(REG_BUFFER, 0)]));
        let mut got = Vec::new();
        for _ in 0..200 {
            got.extend(h.read());
        }
        got
    }

    #[test]
    fn a_capture_goes_through_its_stages_and_dumps() {
        let mut h = device(&[]);
        assert_eq!(status(&mut h), [0x21]);
        arm(&mut h);
        assert_eq!(status(&mut h), [0x33]);
        assert_eq!(status(&mut h), [0x34]);
        assert_eq!(status(&mut h), [0x35]);
        assert_eq!(status(&mut h), [0x36]);
        assert_eq!(status(&mut h), [0x36]);
        let got = dump(&mut h);
        assert_eq!(got.len(), DUMP);
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn a_dump_holds_each_analog_and_logic_value_as_both_programs_decode_it() {
        let mut h = device(&[("pre", "0"), ("wait", "0"), ("post", "0")]);
        arm(&mut h);
        let got = dump(&mut h);
        let mut analog = vec![false; 1024];
        let mut logic = vec![false; 256];
        for (i, s) in got.as_chunks::<3>().0.iter().enumerate() {
            let a = u16::from(s[0] & 0x3f) | u16::from(s[1] & 0x0f) << 6;
            let l = (s[1] & 0x30) >> 4 | (s[2] & 0x3f) << 2;
            assert_eq!((a, l), sample(i));
            analog[usize::from(a)] = true;
            logic[usize::from(l)] = true;
        }
        assert!(analog.iter().all(|x| *x));
        assert!(logic.iter().all(|x| *x));
    }

    #[test]
    fn only_raw_holds_the_flow_control_bytes() {
        assert!(!samples(false).iter().any(|b| matches!(b, 0x11 | 0x13)));
        let raw = samples(true);
        assert!(raw.contains(&0x11) && raw.contains(&0x13));
    }

    #[test]
    fn a_reset_of_the_fsm_ends_a_capture_and_stages_can_be_skipped() {
        let mut h = device(&[("pre", "0"), ("post", "0")]);
        arm(&mut h);
        assert_eq!(status(&mut h), [0x34]);
        h.write(&frame(&[word(REG_CTL1, 0x11)]));
        assert_eq!(status(&mut h), [0x21]);
        let mut h = device(&[("wait", "never"), ("pre", "0")]);
        arm(&mut h);
        for _ in 0..5 {
            assert_eq!(status(&mut h), [0x34]);
        }
        // A forced trigger, which C has and does not call.
        h.write(&frame(&[word(REG_CTL1, 0x18)]));
        assert_eq!(status(&mut h), [0x35]);
        assert_eq!(status(&mut h), [0x36]);
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn a_status_fault_comes_at_its_read() {
        let mut h = device(&[("status-at", "2:ff")]);
        assert_eq!(status(&mut h), [0x21]);
        assert_eq!(status(&mut h), [0xff]);
        assert_eq!(status(&mut h), [0x21]);

        let mut h = device(&[("mute-at", "2")]);
        assert_eq!(status(&mut h), [0x21]);
        assert!(!h.cdc.mark());
        assert_eq!(status(&mut h), b"");
        assert!(h.cdc.mark());

        let mut h = device(&[("late-at", "1:3")]);
        assert_eq!(status(&mut h), b"");
        h.read();
        h.read();
        let mut got = h.read();
        got.extend(h.read());
        assert_eq!(got, [0x21]);

        let mut h = device(&[("mark-at", "2")]);
        status(&mut h);
        h.read();
        assert!(!h.cdc.mark());
        assert_eq!(status(&mut h), [0x21]);
        h.read();
        assert!(h.cdc.mark());
    }

    #[test]
    fn a_dump_fault_changes_its_length_or_its_pace() {
        for (options, length) in [
            (&[("dump", "100")][..], 100),
            (&[("extra", "7")][..], DUMP + 7),
            (&[("pause", "1000:2")][..], DUMP),
        ] {
            let mut o = vec![("pre", "0"), ("wait", "0"), ("post", "0")];
            o.extend_from_slice(options);
            let mut h = device(&o);
            arm(&mut h);
            assert_eq!(dump(&mut h).len(), length, "{options:?}");
        }
        let mut h = device(&[("pre", "0"), ("wait", "0"), ("post", "0"), ("dump", "100")]);
        arm(&mut h);
        dump(&mut h);
        assert!(h.cdc.mark());
    }

    #[test]
    fn a_word_is_taken_however_it_is_cut() {
        let mut h = device(&[]);
        for b in frame(&[word(REG_STATUS, 0)]) {
            h.write(&[b]);
        }
        assert_eq!(h.read(), [0x21]);
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn what_is_not_a_frame_is_flagged() {
        let mut h = device(&[]);
        h.write(b"xyz");
        assert_eq!(violations(&mut h), 1);

        let mut h = device(&[]);
        h.write(b"x@LDS~");
        assert_eq!(violations(&mut h), 1);
        h.write(&[word(REG_STATUS, 0)[0], word(REG_STATUS, 0)[1], b'~']);
        assert_eq!(h.read(), [0x21]);

        let mut h = device(&[]);
        h.write(b"@LDS~~");
        assert_eq!(violations(&mut h), 1);

        let mut h = device(&[]);
        h.write(b"@LDS~\x42~");
        assert_eq!(violations(&mut h), 1);

        let mut h = device(&[]);
        h.write(b"@LDS~");
        h.write(&[0x41; 200]);
        assert_eq!(violations(&mut h), 1);
        assert_eq!(h.cdc.model.pending, b"");

        let mut h = device(&[]);
        h.write(b"@LD");
        assert_eq!(violations(&mut h), 0);
        h.set_lines(true, true);
        assert_eq!(violations(&mut h), 1);
    }

    #[test]
    fn a_word_with_its_fixed_bits_wrong_is_flagged() {
        for bad in [
            [0x42, 0x00], // Bit 6 should be set: value bit 5 is 0.
            [0x42, 0x60], // Bit 6 should be clear: value bit 5 is 1.
            [0x02, 0x40], // Bit 14 should be set: value bit 7 is 0.
            [0x72, 0x40], // Bit 14 should be clear: value bit 7 is 1.
            [0xc2, 0x40], // Bit 15.
            [0x42, 0xc0], // Bit 7.
        ] {
            let mut h = device(&[]);
            h.write(&frame(&[bad]));
            assert_eq!(violations(&mut h), 1, "{bad:02x?}");
            assert_eq!(h.read(), b"", "{bad:02x?}");
        }
    }

    #[test]
    fn a_register_out_of_place_is_flagged() {
        for (words, why) in [
            (vec![word(REG_STATUS, 1)], "a status read with a value"),
            (vec![word(REG_BUFFER, 1)], "a dump read with a value"),
            (vec![word(REG_BUFFER, 0)], "a dump before the data"),
            (vec![word(0, 0)], "register 0 of bank 0"),
            (vec![word(REG_CTL2, 0x40)], "a bit of REG_CTL2"),
            (vec![word(REG_CTL2, 1)], "bank 1"),
            (vec![word(REG_CTL2, 3)], "bank 3"),
            (vec![word(REG_CTL2, 2), word(9, 0)], "register 9 of bank 2"),
            (vec![word(REG_CTL2, 2), word(REG_STATUS + 7, 0)], "another of bank 2"),
            (vec![word(REG_CTL1, 0x04)], "bit 2 of REG_CTL1"),
            (vec![word(REG_CTL1, 0x02)], "an arm with the ADC off"),
            (vec![word(REG_DAC_MSB, 0x10), word(REG_CTL1, 0x20)], "bit 12 of the DAC"),
        ] {
            let mut h = device(&[]);
            h.write(&frame(&words));
            assert!(violations(&mut h) >= 1, "{why}");
        }
    }

    #[test]
    fn what_c_sends_is_taken() {
        let mut h = device(&[]);
        // The ADC reset of an open, then each register of a start.
        h.write(&frame(&[word(REG_CTL2, 0), word(REG_CTL1, 0x40), word(REG_CTL1, 0)]));
        h.write(&frame(&[word(9, 0x9f), word(10, 0x0e)]));
        h.write(&frame(&[word(12, 0x81), word(13, 0x57), word(14, 0xb0)]));
        h.write(&frame(&[word(12, 0x8f), word(13, 0xff), word(14, 0xb0)]));
        h.write(&frame(&[
            word(3, 0),
            word(4, 2),
            word(5, 0),
            word(6, 0xff),
            word(7, 0),
            word(8, 2),
            word(11, 0),
            word(REG_CTL2, 0x22),
            word(0, 0),
            word(1, 0),
            word(2, 0),
            word(3, 0),
            word(4, 0xff),
            word(5, 0xff),
            word(6, 0xff),
            word(7, 0xff),
            word(8, 0),
            word(REG_CTL2, 0x20),
        ]));
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn a_reset_goes_back_to_power_on() {
        let mut h = device(&[("status-at", "2:ff")]);
        arm(&mut h);
        status(&mut h);
        h.write(b"@LD");
        h.cdc.model.reset();
        assert_eq!(h.cdc.model.pending, b"");
        assert_eq!(status(&mut h), [0x21]);
        assert_eq!(status(&mut h), [0xff]);
    }

    #[test]
    fn options_parse() {
        for bad in [
            ("pre", "x"),
            ("wait", "sometimes"),
            ("status-at", "1"),
            ("status-at", "1:xy"),
            ("late-at", "1"),
            ("dump", "3073"),
            ("pause", "3073:1"),
            ("raw", "2"),
            ("vid", "65536"),
            ("nothing", "1"),
        ] {
            let o = vec![(bad.0.to_owned(), bad.1.to_owned())];
            assert!(build(&o).is_err(), "{bad:?}");
        }
        for ok in [("chunk", "1"), ("wait", "never"), ("serial", "x"), ("dump", "0")] {
            let o = vec![(ok.0.to_owned(), ok.1.to_owned())];
            assert!(build(&o).is_ok(), "{ok:?}");
        }
    }
}
