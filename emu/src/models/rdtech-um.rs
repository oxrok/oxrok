// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! An `RDTech` UM24C, UM25C or UM34C USB power meter on the CDC-ACM shell: a
//! one byte request, a 130 byte status dump, no terminators.
//!
//! The device, and how sure this is:
//!
//! - The link: the real meters have no USB data side at all. The "C" models
//!   talk serial over Bluetooth (RFCOMM, a DX-BT18 or a BK3231 module), and
//!   the module talks to the main board at 9600/8n1 (sigrok wiki, "`RDTech` UM
//!   series"). So this USB function, its ids and its strings are an emulation
//!   artifact, a stand-in for `/dev/rfcommN`. The line settings do nothing
//!   there, as on an RFCOMM tty. The `bt/rfcomm/MAC` form of `conn=` is not
//!   reached by any case.
//! - The request: 0xF0 asks for a dump, and is the only command with an
//!   answer (wiki). The others (screens, group selection, thresholds,
//!   backlight, screensaver) exist on the real device; no path of the driver
//!   this model serves sends them, so they are flagged and do nothing.
//! - The dump, big endian (wiki's table): model id at 0, voltage at 2, current
//!   at 4, power at 6, temperatures at 10 and 12, the selected group at 14,
//!   ten groups of mAh and mWh from 16, D+ at 96, D- at 98, charging mode at
//!   100, threshold mAh at 102, threshold mWh at 106, threshold at 110,
//!   duration at 112, active at 116, screen timeout at 118, backlight at 120,
//!   resistance at 122, screen at 126.
//! - The units (wiki): voltage in mV on the UM25C and in 10 mV on the others,
//!   current in 0.1 mA on the UM25C and in mA on the others, D+ and D- in
//!   10 mV on all.
//! - The ids: 0x0963, 0x09C9, 0x0D4C (wiki and the driver agree).
//! - The last two bytes. UM24C and UM25C: the driver wants 0xFF 0xF1; the wiki
//!   has seen 0xF1 in the last on all units and calls both unknown. UM34C: the
//!   last byte is the XOR of 32 odd offsets, whose list the wiki and the driver
//!   give; both bytes drift between polls and start over at a reset (wiki). How
//!   byte 128 drifts is not known: here it counts the answers.
//!
//! Not modelled: the 0xFF the wiki says comes about 500 ms after power on,
//! as a tool is not connected yet then (`noise=1:ff` plays it); the wait of
//! about 0.2 s the wiki asks for after another command, as none is taken.
//!
//! Emulation artifacts:
//!
//! - It answers at once, in the round of the request. Over the real 9600 baud
//!   link 130 bytes take about 135 ms, longer than the driver's 100 ms between
//!   requests, which is why it holds back while a reply is coming. `chunk=N`
//!   cuts the answer up, and `defer=N` plays a reply so late that the host asks
//!   again first. A model has no clock, so nothing here checks the pacing.
//! - Every value but the energy stands still unless an option plays a list. The
//!   energy grows by 1 mWh per answer, to show frames apart.
//!
//! Options, besides the shell's ([`Faults`]):
//!
//! - `model=um24c|um25c|um34c`;
//! - `id=HEX`, two bytes in the id's place, a meter no table has;
//! - `volts=`, `amps=`, `dplus=`, `dminus=`, `celsius=`, `energy=`, `N[,N...]`:
//!   raw register values played in turn, in the model's units;
//! - `answers=N`, polls answered per power on, then silence and the event
//!   point;
//!
//! and, for one answer, `N` counting from 1 per power on (a scan's probe is
//! the first), repeatable:
//!
//! - `bad-end=N`, a wrong last byte;
//! - `short=N:LEN`, only the first LEN bytes;
//! - `noise=N:HEX`, bytes ahead of it;
//! - `junk=N:LEN`, LEN bytes with no id in place of it;
//! - `defer=N`, sent only behind the next answer, which is a link too slow for
//!   the host's pace;
//! - `gap=N:LEN`, its first LEN bytes at once and the rest [`GAP_TICKS`] offers
//!   of a read later, which is a link that pauses. `chunk=` cuts an answer as
//!   finely but in no fixed time, so how a tool's reads fall on it is a race.
//!
//! The pedantic check: every byte the host writes is 0xF0.
//!
//! What `--gcov` leaves unreached in the C, none of it for want of a fault
//! here: `api.c:99`, `api.c:117`, and in `protocol.c` lines 67 and 88 (the
//! length is always the dump's), 202-203 (likewise), 216 (the offsets fit),
//! 260, 341 and 343 cannot happen; 219 needs an output that fails; 131-132
//! need a port that fails between its open and its first write, with no event
//! point in between; 255 is a read that fails, and a hung up port is not read
//! at all, as its events are not plain `G_IO_IN` (`protocol.c:350`).

use crate::model::DeviceModel;
use crate::options::{parse_hex_list, parse_number};
use crate::shells::cdc::{Cdc, Faults, Identity, Port, Serial};
use crate::{Error, Result};

/// The model name.
pub const NAME: &str = "rdtech-um";
/// What `oxrok-emu list` says about it.
pub const ABOUT: &str = "An RDTech UM24C, UM25C or UM34C power meter, CDC-ACM standing in for its \
                         Bluetooth link; model=, id=, volts=, amps=, dplus=, dminus=, celsius=, \
                         energy=, answers=, bad-end=, short=, noise=, junk=, defer=, gap=, and the \
                         shell's stall=, chunk=, in-limit=, in-stall=, out-hold=";

/// The one command this model takes.
const POLL: u8 = 0xf0;
/// A dump's length.
const DUMP: usize = 130;
/// A byte no id starts or ends with, for `junk=`.
const JUNK: u8 = 0x55;
/// The most bytes `junk=` and `noise=` take: more than any host buffer.
const MOST_FAULT_BYTES: u32 = 4096;
/// Offers of a read that `gap=` waits, each about 20 ms (see `Serial::tick`).
const GAP_TICKS: u32 = 5;
/// The offsets the UM34C's last byte covers.
const UM34C_SUM: [usize; 32] = [
    1, 3, 7, 9, 15, 17, 19, 23, 31, 39, 41, 45, 49, 53, 55, 57, 59, 63, 67, 69, 73, 79, 83, 89, 97,
    99, 109, 111, 113, 119, 121, 127,
];

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Kind {
    Um24c,
    Um25c,
    Um34c,
}

impl Kind {
    fn id(self) -> [u8; 2] {
        match self {
            Kind::Um24c => [0x09, 0x63],
            Kind::Um25c => [0x09, 0xc9],
            Kind::Um34c => [0x0d, 0x4c],
        }
    }
}

/// What happens to one answer.
#[derive(Clone, Debug, PartialEq, Eq)]
enum Fault {
    BadEnd,
    Short(usize),
    Noise(Vec<u8>),
    Junk(usize),
    Defer,
    Gap(usize),
}

#[derive(Clone, Debug)]
struct Config {
    kind: Kind,
    id: Option<[u8; 2]>,
    volts: Vec<u32>,
    amps: Vec<u32>,
    dplus: Vec<u32>,
    dminus: Vec<u32>,
    celsius: Vec<u32>,
    energy: Vec<u32>,
    answers: Option<u32>,
    faults: Vec<(u32, Fault)>,
}

fn list(key: &str, value: &str, most: u32) -> Result<Vec<u32>> {
    value.split(',').map(|v| parse_number(key, v, 0..=most)).collect()
}

/// `N:REST`, with N from 1.
fn numbered<'a>(key: &str, value: &'a str) -> Result<(u32, &'a str)> {
    let (n, rest) =
        value.split_once(':').ok_or_else(|| Error(format!("{key}={value}: as in 2:{key}")))?;
    Ok((parse_number(key, n, 1..=1_000_000)?, rest))
}

/// A meter from the `-o` options.
///
/// # Errors
///
/// On an unknown option or a value out of range.
pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>> {
    Ok(Box::new(function(options)?))
}

fn function(options: &[(String, String)]) -> Result<Cdc<Meter>> {
    const U16: u32 = 0xffff;
    let mut shell = Faults::default();
    let mut c = Config {
        kind: Kind::Um24c,
        id: None,
        volts: Vec::new(),
        amps: Vec::new(),
        dplus: vec![59],
        dminus: vec![58],
        celsius: vec![27],
        energy: Vec::new(),
        answers: None,
        faults: Vec::new(),
    };
    for (k, v) in options {
        if shell.option(k, v)? {
            continue;
        }
        match k.as_str() {
            "model" => {
                c.kind = match v.as_str() {
                    "um24c" => Kind::Um24c,
                    "um25c" => Kind::Um25c,
                    "um34c" => Kind::Um34c,
                    _ => return Err(Error(format!("model={v}: um24c, um25c or um34c"))),
                };
            }
            "id" => {
                let id = parse_hex_list(k, v, 2, 2)?.remove(0);
                c.id = Some([id[0], id[1]]);
            }
            "volts" => c.volts = list(k, v, U16)?,
            "amps" => c.amps = list(k, v, U16)?,
            "dplus" => c.dplus = list(k, v, U16)?,
            "dminus" => c.dminus = list(k, v, U16)?,
            "celsius" => c.celsius = list(k, v, U16)?,
            "energy" => c.energy = list(k, v, u32::MAX)?,
            "answers" => c.answers = Some(parse_number(k, v, 0..=1_000_000)?),
            "bad-end" => c.faults.push((parse_number(k, v, 1..=1_000_000)?, Fault::BadEnd)),
            "defer" => c.faults.push((parse_number(k, v, 1..=1_000_000)?, Fault::Defer)),
            "short" => {
                let (n, len) = numbered(k, v)?;
                let len = parse_number(k, len, 0..=u32::try_from(DUMP).expect("130"))?;
                c.faults.push((n, Fault::Short(len as usize)));
            }
            "junk" => {
                let (n, len) = numbered(k, v)?;
                c.faults.push((n, Fault::Junk(parse_number(k, len, 1..=MOST_FAULT_BYTES)? as usize)));
            }
            "gap" => {
                let (n, len) = numbered(k, v)?;
                let len = parse_number(k, len, 1..=MOST_FAULT_BYTES)? as usize;
                c.faults.push((n, Fault::Gap(len)));
            }
            "noise" => {
                let (n, hex) = numbered(k, v)?;
                let bytes = parse_hex_list(k, hex, 1, MOST_FAULT_BYTES as usize)?.remove(0);
                c.faults.push((n, Fault::Noise(bytes)));
            }
            other => return Err(Error(format!("{NAME} has no option {other}"))),
        }
    }
    // pid.codes' test id: no such USB device exists.
    let id = Identity::new(0x1209, 0x0001, "oxrok-emu", "RDTech UM link");
    Ok(Cdc::new(NAME, &id, shell, Meter::new(c)))
}

/// The value for answer `i` (from 0): the list's, in turn, or `default`.
fn in_turn(values: &[u32], i: u32, default: u32) -> u32 {
    if values.is_empty() { default } else { values[i as usize % values.len()] }
}

struct Meter {
    config: Config,
    /// Polls taken since power on.
    polls: u32,
    /// An answer held back by `defer=`.
    held: Vec<u8>,
    /// The rest of an answer under `gap=`, and the offers it still waits.
    rest: Vec<u8>,
    rest_ticks: u32,
}

impl Meter {
    fn new(config: Config) -> Meter {
        Meter { config, polls: 0, held: Vec::new(), rest: Vec::new(), rest_ticks: 0 }
    }

    /// The dump for answer `i` (from 0), as sound as the device makes it.
    fn dump(&self, i: u32) -> [u8; DUMP] {
        let c = &self.config;
        let (volts_at_rest, amps_at_rest) =
            if c.kind == Kind::Um25c { (5123, 2501) } else { (512, 250) };
        let volts = in_turn(&c.volts, i, volts_at_rest);
        let amps = in_turn(&c.amps, i, amps_at_rest);
        let celsius = in_turn(&c.celsius, i, 27);
        let power_scale = if c.kind == Kind::Um25c { 10_000 } else { 100 };
        let power = u64::from(volts) * u64::from(amps) / power_scale;
        let ohms = (u64::from(volts) * 100).checked_div(u64::from(amps)).unwrap_or(0);

        let mut d = [0u8; DUMP];
        let mut put16 = |at: usize, v: u32| {
            d[at..at + 2].copy_from_slice(&(v as u16).to_be_bytes());
        };
        put16(2, volts);
        put16(4, amps);
        put16(10, celsius);
        put16(12, celsius * 9 / 5 + 32);
        put16(96, in_turn(&c.dplus, i, 59));
        put16(98, in_turn(&c.dminus, i, 58));
        put16(110, 30);
        put16(116, 1);
        put16(118, 2);
        put16(120, 3);
        d[..2].copy_from_slice(&c.id.unwrap_or(c.kind.id()));
        d[6..10].copy_from_slice(&u32::try_from(power).unwrap_or(u32::MAX).to_be_bytes());
        let energy = in_turn(&c.energy, i, 1234u32.wrapping_add(i));
        d[106..110].copy_from_slice(&energy.to_be_bytes());
        d[122..126].copy_from_slice(&u32::try_from(ohms).unwrap_or(u32::MAX).to_be_bytes());
        if c.kind == Kind::Um34c {
            d[128] = i as u8;
            d[129] = UM34C_SUM.iter().fold(0, |sum, &at| sum ^ d[at]);
        } else {
            d[128] = 0xff;
            d[129] = 0xf1;
        }
        d
    }

    /// Answer `n` (from 1) as the faults leave it: the bytes, whether they wait
    /// for the next answer, and how many go ahead of a gap.
    fn answer(&self, n: u32) -> (Vec<u8>, bool, Option<usize>) {
        let mut bytes = self.dump(n - 1).to_vec();
        let mut defer = false;
        let mut gap = None;
        for (_, fault) in self.config.faults.iter().filter(|(at, _)| *at == n) {
            match fault {
                Fault::BadEnd => bytes[DUMP - 1] ^= 1,
                Fault::Short(len) => bytes.truncate(*len),
                Fault::Noise(noise) => {
                    bytes.splice(0..0, noise.iter().copied());
                }
                Fault::Junk(len) => bytes = vec![JUNK; *len],
                Fault::Defer => defer = true,
                Fault::Gap(len) => gap = Some(*len),
            }
        }
        (bytes, defer, gap)
    }

    fn poll(&mut self, port: &mut Port<'_>) {
        if self.config.answers.is_some_and(|n| self.polls >= n) {
            port.mark_event_point();
            return;
        }
        self.polls += 1;
        let (mut bytes, defer, gap) = self.answer(self.polls);
        if defer {
            self.held.extend(bytes);
            return;
        }
        let held = std::mem::take(&mut self.held);
        port.send(&held);
        if let Some(len) = gap.filter(|len| *len < bytes.len()) {
            self.rest = bytes.split_off(len);
            self.rest_ticks = GAP_TICKS;
        }
        port.send(&bytes);
    }
}

impl Serial for Meter {
    fn received(&mut self, data: &[u8], port: &mut Port<'_>) {
        for &byte in data {
            if byte == POLL {
                self.poll(port);
            } else {
                port.violation(format!("command {byte:#04x}: only 0xf0 is ever asked"));
            }
        }
    }

    fn tick(&mut self, port: &mut Port<'_>) {
        if self.rest.is_empty() {
            return;
        }
        self.rest_ticks = self.rest_ticks.saturating_sub(1);
        if self.rest_ticks == 0 {
            port.send(&std::mem::take(&mut self.rest));
        }
    }

    fn reset(&mut self) {
        *self = Meter::new(self.config.clone());
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::shells::cdc::Line;
    use crate::model::Reply;
    use crate::shells::cdc::host::Host;

    fn meter(options: &[(&str, &str)]) -> Host<Meter> {
        let o: Vec<(String, String)> =
            options.iter().map(|(k, v)| ((*k).to_owned(), (*v).to_owned())).collect();
        Host::open(function(&o).unwrap(), Line::default(), true, true)
    }

    fn poll(h: &mut Host<Meter>) -> Vec<u8> {
        h.write(&[POLL]);
        h.read()
    }

    fn u16_at(d: &[u8], at: usize) -> u16 {
        u16::from_be_bytes([d[at], d[at + 1]])
    }

    fn u32_at(d: &[u8], at: usize) -> u32 {
        u32::from_be_bytes([d[at], d[at + 1], d[at + 2], d[at + 3]])
    }

    #[test]
    fn a_dump_has_its_fields_where_the_table_says() {
        let mut h = meter(&[]);
        let d = poll(&mut h);
        assert_eq!(d.len(), DUMP);
        assert_eq!(u16_at(&d, 0), 0x0963);
        assert_eq!(u16_at(&d, 2), 512);
        assert_eq!(u16_at(&d, 4), 250);
        assert_eq!(u32_at(&d, 6), 1280, "5.12 V at 0.25 A, in mW");
        assert_eq!(u16_at(&d, 10), 27);
        assert_eq!(u16_at(&d, 12), 80);
        assert_eq!(u16_at(&d, 96), 59);
        assert_eq!(u16_at(&d, 98), 58);
        assert_eq!(u32_at(&d, 106), 1234);
        assert_eq!(u32_at(&d, 122), 204, "20.4 ohms, in tenths");
        assert_eq!(u16_at(&d, 128), 0xfff1);
        assert_eq!(h.v.drain().len(), 0);
    }

    #[test]
    fn the_um25c_counts_in_finer_units() {
        let mut h = meter(&[("model", "um25c")]);
        let d = poll(&mut h);
        assert_eq!(u16_at(&d, 0), 0x09c9);
        assert_eq!(u16_at(&d, 2), 5123);
        assert_eq!(u16_at(&d, 4), 2501);
        assert_eq!(u32_at(&d, 6), 1281, "5.123 V at 0.2501 A, in mW");
        assert_eq!(u32_at(&d, 122), 204);
        assert_eq!(u16_at(&d, 128), 0xfff1);
    }

    #[test]
    fn the_um34c_sums_its_odd_offsets_and_drifts() {
        let mut h = meter(&[("model", "um34c")]);
        for i in 0..3 {
            let d = poll(&mut h);
            assert_eq!(u16_at(&d, 0), 0x0d4c);
            assert_eq!(d[128], i, "byte 128 counts the answers");
            let sum = UM34C_SUM.iter().fold(0, |s, &at| s ^ d[at]);
            assert_eq!(d[129], sum);
        }
        assert!(UM34C_SUM.iter().all(|at| at % 2 == 1));
    }

    #[test]
    fn the_energy_tells_frames_apart() {
        let mut h = meter(&[]);
        assert_eq!(u32_at(&poll(&mut h), 106), 1234);
        assert_eq!(u32_at(&poll(&mut h), 106), 1235);
    }

    #[test]
    fn lists_play_in_turn_and_start_over() {
        let mut h = meter(&[("volts", "100,200"), ("energy", "4294967295")]);
        let d = poll(&mut h);
        assert_eq!(u16_at(&d, 2), 100);
        assert_eq!(u32_at(&d, 106), u32::MAX);
        assert_eq!(u16_at(&poll(&mut h), 2), 200);
        assert_eq!(u16_at(&poll(&mut h), 2), 100);
    }

    #[test]
    fn two_polls_in_one_write_get_two_answers() {
        let mut h = meter(&[]);
        h.write(&[POLL, POLL]);
        assert_eq!(h.read().len(), 2 * DUMP);
        assert_eq!(h.v.drain().len(), 0);
    }

    #[test]
    fn answers_runs_out_and_a_reset_brings_it_back() {
        let mut h = meter(&[("answers", "1")]);
        assert_eq!(poll(&mut h).len(), DUMP);
        assert!(!h.cdc.take_event_point());
        assert_eq!(poll(&mut h).len(), 0);
        assert!(h.cdc.take_event_point());
        h.cdc.model.reset();
        assert_eq!(poll(&mut h).len(), DUMP);
        let mut h = meter(&[("answers", "0")]);
        assert_eq!(poll(&mut h).len(), 0);
    }

    #[test]
    fn a_fault_hits_its_own_answer_only() {
        let mut h = meter(&[("bad-end", "2"), ("short", "3:10"), ("junk", "4:5"), ("noise", "5:abcd")]);
        assert_eq!(u16_at(&poll(&mut h), 128), 0xfff1);
        let d = poll(&mut h);
        assert_eq!(d.len(), DUMP);
        assert_eq!(u16_at(&d, 128), 0xfff0);
        assert_eq!(poll(&mut h).len(), 10);
        assert_eq!(poll(&mut h), [JUNK; 5]);
        let d = poll(&mut h);
        assert_eq!(d.len(), DUMP + 2);
        assert_eq!(d[..4], [0xab, 0xcd, 0x09, 0x63]);
        assert_eq!(poll(&mut h).len(), DUMP);
    }

    #[test]
    fn a_wrong_id_keeps_the_rest() {
        let mut h = meter(&[("id", "1234")]);
        let d = poll(&mut h);
        assert_eq!(u16_at(&d, 0), 0x1234);
        assert_eq!(u16_at(&d, 128), 0xfff1);
    }

    #[test]
    fn a_deferred_answer_comes_behind_the_next_poll() {
        let mut h = meter(&[("defer", "2")]);
        assert_eq!(poll(&mut h).len(), DUMP);
        assert_eq!(poll(&mut h).len(), 0);
        let both = poll(&mut h);
        assert_eq!(both.len(), 2 * DUMP);
        assert_eq!(u32_at(&both, 106), 1235, "the held one first");
        assert_eq!(u32_at(&both, DUMP + 106), 1236);
        let mut h = meter(&[("defer", "1")]);
        assert_eq!(poll(&mut h).len(), 0);
        h.cdc.model.reset();
        assert_eq!(poll(&mut h).len(), 0, "the held one went with the reset, this is deferred too");
        assert_eq!(poll(&mut h).len(), 2 * DUMP);
    }

    #[test]
    fn a_gap_sends_the_rest_after_its_offers_and_no_sooner() {
        let mut h = meter(&[("gap", "1:3")]);
        h.write(&[POLL]);
        assert!(matches!(h.offer(), Reply::Data(d) if d.len() == 3));
        for _ in 1..GAP_TICKS {
            assert!(!matches!(h.offer(), Reply::Data(_)), "not before its offers");
        }
        let rest = h.read();
        assert_eq!(rest.len(), DUMP - 3);
        assert_eq!(rest[0], 0x00, "byte 3 on: the low half of the voltage");
        assert_eq!(poll(&mut h).len(), DUMP, "one answer only");
        assert_eq!(h.v.drain().len(), 0);
    }

    #[test]
    fn any_other_command_is_flagged_and_gets_nothing() {
        for command in [0xf1, 0xf4, 0xa0, 0xb3, 0xd2, 0xe1, 0x00, 0xff] {
            let mut h = meter(&[]);
            h.write(&[command]);
            assert_eq!(h.read().len(), 0, "{command:#x}");
            assert_eq!(h.v.drain().len(), 1, "{command:#x}");
        }
        let mut h = meter(&[]);
        h.write(&[0xf1, POLL]);
        assert_eq!(h.read().len(), DUMP, "a poll behind it is still one");
        assert_eq!(h.v.drain().len(), 1);
    }

    #[test]
    fn options_are_checked() {
        for (k, v) in [
            ("model", "um99"),
            ("id", "12"),
            ("volts", "65536"),
            ("short", "2"),
            ("short", "0:1"),
            ("short", "1:131"),
            ("junk", "1:0"),
            ("noise", "1:abc"),
            ("defer", "0"),
            ("gap", "1:0"),
            ("gap", "1"),
            ("nosuch", "1"),
        ] {
            let o = [(k.to_owned(), v.to_owned())];
            assert!(function(&o).is_err(), "{k}={v}");
        }
    }
}
