// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! The Kecheng KC-330B sound level meter and data logger: a Silicon Labs
//! F320 behind one vendor interface, with bulk OUT 2 for commands and
//! bulk IN 0x81 for answers, and a 24LC512 EEPROM for its log.
//!
//! Sources, and how sure each is:
//! - The descriptors are the sigrok wiki's `lsusb -vvv` of a real meter
//!   (`Kecheng_KC-330B/Info`): `1041:8101`, bcdUSB 2.00, bcdDevice 1.00,
//!   a 64 byte endpoint 0, no strings, bus powered at 20 mA, interface
//!   class ff/01/02, both bulk endpoints 64 bytes. The wiki's output does
//!   not say the speed; 64 byte bulk packets mean full speed, the F320's.
//! - The commands are the wiki's protocol table (`Kecheng_KC-330B`) and
//!   the C driver (`protocol.c`, `protocol.h`), which agree except as
//!   said below. No firmware or vendor source was found, and no recording
//!   was examined: a C claim about the device is only that.
//!
//! | command | length | answer |
//! |---|---|---|
//! | `01` configure | 7 | `81` |
//! | `02` identify | 1 | `82`, length, ASCII model |
//! | `03` set date and time | 7 | `83` |
//! | `04` status | 1 | `84` activated, `a4` not |
//! | `05` log info | 1 | `85`, the six configuration bytes, count (BE) |
//! | `06` log start time | 1 | `86`, the six date and time bytes |
//! | `07` log data | 4 | `87`, then 2 BE bytes per sample |
//! | `08` live level | 1 | `88`, 2 BE bytes: dB times 10 |
//!
//! Configuration bytes: interval index 0 to 6 (125 ms, 500 ms, 1 s, 2 s,
//! 5 s, 10 s, 60 s), alarm low, alarm high, time weighting (0 F, 1 S),
//! frequency weighting (0 A, 1 C), data source. Date bytes: year in the
//! century, month, day, hours, minutes, seconds. A log request is an
//! offset in blocks of 63 samples (BE) and a count, 63 until the last
//! chunk; at most 32000 samples are stored (all from the wiki).
//!
//! What is not settled, and what the model does about it:
//! - The data source byte: the wiki has 0 memory, 1 real time, while the
//!   C sends 0 for live and 1 for memory (`protocol.h:59`, `protocol.c:209`).
//!   Nothing here depends on it, so both pass, and it is only stored.
//! - Configure clears the log: only the C's comment says so
//!   (`api.c:430`). The model follows it, and no case can show a clearing
//!   as a fact about a real meter.
//! - An inactive meter reads 30.0 dB until its button is pressed: the C's
//!   comment again (`api.c:345`). There is no button here: `inactive=1`
//!   stays so.
//! - The wiki's "(except 0x11...)" on the log info's configuration bytes
//!   is unexplained. The model answers the bytes the log was taken with
//!   and `log-source=` sets the last; the C reads only the interval and
//!   the two weightings.
//! - The model string is on no page: `KC-330B` is the product's name,
//!   not an observed answer. `model=` changes it.
//!
//! Emulation artifacts:
//! - Every answer is ready at once. The C says a configure ack takes
//!   about 32 ms (`protocol.c:216`).
//! - Unread answers queue in order, up to [`QUEUE`]. A real F320 has one
//!   IN FIFO, and what its firmware does with an answer nobody read is
//!   unknown. This shows when the host abandons an answer: the C's log
//!   start does (`api.c:405`), so its following configure gets the log
//!   chunk in a one byte read, a babble.
//! - The levels are stand-ins, one per request, so the samples do not
//!   depend on timing: [`live_level`] and [`stored_level`].
//! - The clock does not run; the start time in the `06` answer is a
//!   stand-in, as the log's start is on no source.
//! - A one-shot STALL (`stall-in=`, `stall-out=`, with `@N`) ends by
//!   itself; a real STALL holds until a `CLEAR_FEATURE`.
//!
//! Options, none faulty by default: `model=TEXT`, `inactive=1`,
//! `stored=N` (0 to 32000, default 100), `log-interval=I` (0 to 7; 7 is
//! one the C refuses, as a corrupt EEPROM could hold),
//! `log-time=0|1`, `log-freq=0|1`, `log-source=B`, `id-length=N` (the
//! identify's length byte, whatever the model string is), `digest=1` (log
//! chunks logged as digests). The faults take a command in two hex
//! digits and the Nth time it comes, or every time without `@N`:
//! `stall-out=CC[@N]` STALLs the command (no effect), `silent=CC[@N]`
//! takes it and does not answer, `stall-in=CC[@N]` STALLs the answer,
//! `short=CC[@N]` drops the answer's last byte (one byte answers become
//! an empty packet), `garbage=CC[@N]` flips bit 6 of its first byte.

use crate::descriptor::{self, DeviceDescriptor};
use crate::model::{Descriptors, DeviceModel, Reply, Setup, Speed, Violations};
use crate::{Error, Result};
use std::collections::VecDeque;

/// The model name.
pub const NAME: &str = "kecheng-kc-330b";
/// What `oxrok-emu list` says about it.
pub const ABOUT: &str = "Kecheng KC-330B sound level meter (vendor bulk, 1041:8101); model=TEXT, \
     inactive=1, stored=N, log-interval=I, log-time=0|1, log-freq=0|1, log-source=B, id-length=N, \
     digest=1, faults: stall-out=CC[@N], silent=CC[@N], stall-in=CC[@N], short=CC[@N], \
     garbage=CC[@N]";

/// The USB vendor id, from the wiki's lsusb.
pub const VENDOR_ID: u16 = 0x1041;
/// The USB product id.
pub const PRODUCT_ID: u16 = 0x8101;
/// The bulk endpoint commands arrive on.
pub const EP_OUT: u8 = 2;
/// The bulk endpoint answers leave from, without the direction bit.
pub const EP_IN: u8 = 1;
/// Samples in a block of the log, the most a request can ask for.
pub const BLOCK: usize = 63;
/// The most samples the log holds (the wiki).
pub const MAX_STORED: u16 = 32000;
/// The model string; see the module documentation.
pub const MODEL: &str = "KC-330B";
/// Unread answers kept; one more is a violation.
pub const QUEUE: usize = 8;
/// The `06` answer's stand-in: 2013-08-03 12:00:00.
pub const LOG_START: [u8; 6] = [13, 8, 3, 12, 0, 0];
/// With `digest=1`, data over this is logged as a digest: a chunk of 32 or
/// more samples, the whole log's being many.
pub const DIGEST_ABOVE: usize = 64;
/// What an inactive meter reads, dB times 10 (`api.c:347`).
pub const INACTIVE_LEVEL: u16 = 300;

const CONFIGURE: u8 = 0x01;
const IDENTIFY: u8 = 0x02;
const SET_DATE_TIME: u8 = 0x03;
const STATUS: u8 = 0x04;
const LOG_INFO: u8 = 0x05;
const LOG_START_TIME: u8 = 0x06;
const LOG_DATA: u8 = 0x07;
const LIVE: u8 = 0x08;

/// The stand-in level of the `n`th live request (from 0), dB times 10:
/// inside the meter's 30 to 130 dB.
pub fn live_level(n: u32) -> u16 {
    300 + (n.wrapping_mul(37) % 1000) as u16
}

/// The stand-in level of stored sample `i`, dB times 10.
pub fn stored_level(i: usize) -> u16 {
    300 + ((i % 1000) as u16 * 13) % 1000
}

/// When a fault fires: the Nth time its command comes, or every time.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Fault {
    command: u8,
    nth: Option<u32>,
}

impl Fault {
    fn parse(key: &str, value: &str) -> Result<Fault> {
        let bad = || Error::Invalid(format!("{key}={value}: CC or CC@N, CC two hex digits and N from 1"));
        let (command, nth) = match value.split_once('@') {
            Some((c, n)) => (c, Some(n.parse().ok().filter(|n| *n > 0).ok_or_else(bad)?)),
            None => (value, None),
        };
        if command.len() != 2 {
            return Err(bad());
        }
        Ok(Fault { command: u8::from_str_radix(command, 16).map_err(|_| bad())?, nth })
    }

    fn fires(self, command: u8, count: u32) -> bool {
        self.command == command && self.nth.is_none_or(|n| n == count)
    }
}

/// What a fault does to a command.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Action {
    StallOut,
    Silent,
    StallIn,
    Short,
    Garbage,
}

/// An answer waiting for an IN transfer.
#[derive(Clone, Debug, PartialEq, Eq)]
enum Answer {
    Data(Vec<u8>),
    Stall,
}

/// The settings of a configure command, as last taken.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Config {
    /// Interval index, 0 to 6.
    pub interval: u8,
    /// Alarm low threshold.
    pub alarm_low: u8,
    /// Alarm high threshold.
    pub alarm_high: u8,
    /// Time weighting, 0 F, 1 S.
    pub time: u8,
    /// Frequency weighting, 0 A, 1 C.
    pub freq: u8,
    /// Data source; see the module documentation.
    pub source: u8,
}

/// The emulated meter.
pub struct Kc330b {
    descriptors: Descriptors,
    model: Vec<u8>,
    id_length: Option<u8>,
    inactive: bool,
    /// Samples in the log.
    stored: u16,
    log_info: [u8; 6],
    faults: Vec<(Action, Fault)>,
    answers: VecDeque<Answer>,
    /// Times each command came, for `@N`.
    seen: [u32; 256],
    live_reads: u32,
    /// The last configure taken.
    config: Option<Config>,
    /// The last date and time taken.
    clock: Option<[u8; 6]>,
}

/// `digest=1` logs long data as its digest.
pub fn log_rules(options: &[(String, String)]) -> crate::txlog::Rules {
    crate::txlog::Rules {
        digest_above: options.iter().any(|(k, v)| k == "digest" && v == "1").then_some(DIGEST_ABOVE),
        undefined: None,
    }
}

/// A meter from the `-o` options; see the module documentation.
///
/// # Errors
///
/// On an unknown option or a value out of range.
pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>> {
    let mut dev = Kc330b::new();
    for (k, v) in options {
        let byte = |max: u8| -> Result<u8> {
            v.parse().ok().filter(|n| *n <= max).ok_or_else(|| Error::Invalid(format!("{k}={v}: 0 to {max}")))
        };
        match k.as_str() {
            "model" => {
                if v.len() > 60 || !v.is_ascii() {
                    return Err(Error::Invalid(format!("model={v}: ASCII, at most 60 bytes")));
                }
                v.as_bytes().clone_into(&mut dev.model);
            }
            "inactive" => dev.inactive = byte(1)? == 1,
            "stored" => {
                dev.stored = v
                    .parse()
                    .ok()
                    .filter(|n| *n <= MAX_STORED)
                    .ok_or_else(|| Error::Invalid(format!("stored={v}: 0 to {MAX_STORED}")))?;
            }
            "log-interval" => dev.log_info[0] = byte(7)?,
            "log-time" => dev.log_info[3] = byte(1)?,
            "log-freq" => dev.log_info[4] = byte(1)?,
            "log-source" => dev.log_info[5] = byte(255)?,
            "id-length" => dev.id_length = Some(byte(255)?),
            // log_rules reads it; 0 and 1 are all build takes.
            "digest" => {
                byte(1)?;
            }
            "stall-out" => dev.faults.push((Action::StallOut, Fault::parse(k, v)?)),
            "silent" => dev.faults.push((Action::Silent, Fault::parse(k, v)?)),
            "stall-in" => dev.faults.push((Action::StallIn, Fault::parse(k, v)?)),
            "short" => dev.faults.push((Action::Short, Fault::parse(k, v)?)),
            "garbage" => dev.faults.push((Action::Garbage, Fault::parse(k, v)?)),
            other => return Err(Error::Invalid(format!("{NAME} has no option {other}"))),
        }
    }
    Ok(Box::new(dev))
}

impl Default for Kc330b {
    fn default() -> Self {
        Self::new()
    }
}

impl Kc330b {
    /// A meter with a log of 100 samples taken at 1 s, S and C weighted,
    /// which is no one's default, so that a log's settings show.
    pub fn new() -> Kc330b {
        let mut body = Vec::new();
        body.extend_from_slice(&descriptor::interface(0, 0, 2, [0xff, 1, 2], 0));
        body.extend_from_slice(&descriptor::endpoint(0x80 | EP_IN, 2, 64, 0));
        body.extend_from_slice(&descriptor::endpoint(EP_OUT, 2, 64, 0));
        let device = DeviceDescriptor {
            usb: 0x0200,
            class: [0, 0, 0],
            max_packet0: 64,
            vendor: VENDOR_ID,
            product: PRODUCT_ID,
            device: 0x0100,
            strings: [0, 0, 0],
            configurations: 1,
        };
        Kc330b {
            descriptors: Descriptors {
                speed: Speed::Full,
                device: device.bytes(),
                configs: vec![descriptor::configuration(1, 0, 0x80, 20, &body)],
                strings: Vec::new(),
            },
            model: MODEL.as_bytes().to_vec(),
            id_length: None,
            inactive: false,
            stored: 100,
            log_info: [2, 40, 120, 1, 1, 0],
            faults: Vec::new(),
            answers: VecDeque::new(),
            seen: [0; 256],
            live_reads: 0,
            config: None,
            clock: None,
        }
    }

    /// The last configure taken.
    pub fn config(&self) -> Option<Config> {
        self.config
    }

    /// The last date and time taken.
    pub fn clock(&self) -> Option<[u8; 6]> {
        self.clock
    }

    /// Samples in the log.
    pub fn stored(&self) -> u16 {
        self.stored
    }

    fn fault(&self, command: u8, count: u32) -> Option<Action> {
        self.faults.iter().find(|(_, f)| f.fires(command, count)).map(|(a, _)| *a)
    }

    fn configure(&mut self, d: &[u8], v: &mut Violations) -> Vec<u8> {
        let config = Config {
            interval: d[1],
            alarm_low: d[2],
            alarm_high: d[3],
            time: d[4],
            freq: d[5],
            source: d[6],
        };
        if config.interval > 6 {
            v.report(format!("configure with interval index {}, the meter has 0 to 6", d[1]));
        }
        if config.time > 1 {
            v.report(format!("configure with time weighting {}, not 0 or 1", d[4]));
        }
        if config.freq > 1 {
            v.report(format!("configure with frequency weighting {}, not 0 or 1", d[5]));
        }
        if config.source > 1 {
            v.report(format!("configure with data source {}, not 0 or 1", d[6]));
        }
        self.config = Some(config);
        // api.c:430, "it clears it whenever new configuration is set".
        self.stored = 0;
        vec![CONFIGURE | 0x80]
    }

    fn set_date_time(&mut self, d: &[u8], v: &mut Violations) -> Vec<u8> {
        let ok = (1..=12).contains(&d[2])
            && (1..=31).contains(&d[3])
            && d[4] < 24
            && d[5] < 60
            && d[6] < 60;
        if !ok {
            v.report(format!("date and time out of range: {:?}", &d[1..7]));
        }
        let mut clock = [0; 6];
        clock.copy_from_slice(&d[1..7]);
        self.clock = Some(clock);
        vec![SET_DATE_TIME | 0x80]
    }

    fn identify(&self) -> Vec<u8> {
        let length = self.id_length.unwrap_or(self.model.len().min(255) as u8);
        let mut a = vec![IDENTIFY | 0x80, length];
        a.extend_from_slice(&self.model);
        a
    }

    fn log_info(&self) -> Vec<u8> {
        let mut a = vec![LOG_INFO | 0x80];
        a.extend_from_slice(&self.log_info);
        a.extend_from_slice(&self.stored.to_be_bytes());
        a
    }

    fn log_data(&self, d: &[u8], v: &mut Violations) -> Vec<u8> {
        let block = usize::from(u16::from_be_bytes([d[1], d[2]]));
        let count = usize::from(d[3]);
        if count == 0 || count > BLOCK {
            v.report(format!("log request for {count} samples, 1 to {BLOCK} per request"));
        }
        let first = block.saturating_mul(BLOCK);
        let stored = usize::from(self.stored);
        if first.saturating_add(count) > stored {
            v.report(format!(
                "log request for {count} samples at block {block}, the log has {stored}"
            ));
        }
        let mut a = vec![LOG_DATA | 0x80];
        // Only what is stored, and a request's count fits one byte.
        for i in first..first.saturating_add(count).min(stored) {
            a.extend_from_slice(&stored_level(i).to_be_bytes());
        }
        a
    }

    fn live(&mut self) -> Vec<u8> {
        let level = if self.inactive { INACTIVE_LEVEL } else { live_level(self.live_reads) };
        self.live_reads = self.live_reads.wrapping_add(1);
        let b = level.to_be_bytes();
        vec![LIVE | 0x80, b[0], b[1]]
    }

    /// The answer of a whole command; `None` for one the meter does not
    /// have, which is flagged.
    fn answer(&mut self, data: &[u8], v: &mut Violations) -> Option<Vec<u8>> {
        let command = data[0];
        let want = match command {
            CONFIGURE | SET_DATE_TIME => 7,
            IDENTIFY | STATUS | LOG_INFO | LOG_START_TIME | LIVE => 1,
            LOG_DATA => 4,
            other => {
                v.report(format!("command {other:#04x} is not one the meter has"));
                return None;
            }
        };
        if data.len() != want {
            v.report(format!("command {command:#04x} of {} bytes, it has {want}", data.len()));
            return None;
        }
        Some(match command {
            CONFIGURE => self.configure(data, v),
            IDENTIFY => self.identify(),
            SET_DATE_TIME => self.set_date_time(data, v),
            STATUS => vec![if self.inactive { STATUS | 0xa0 } else { STATUS | 0x80 }],
            LOG_INFO => self.log_info(),
            LOG_START_TIME => {
                let mut a = vec![LOG_START_TIME | 0x80];
                a.extend_from_slice(&LOG_START);
                a
            }
            LOG_DATA => self.log_data(data, v),
            _ => self.live(),
        })
    }
}

impl DeviceModel for Kc330b {
    fn name(&self) -> &'static str {
        NAME
    }

    fn descriptors(&self) -> &Descriptors {
        &self.descriptors
    }

    fn control(&mut self, setup: Setup, _data: &[u8], v: &mut Violations) -> Reply {
        v.report(format!(
            "request {:#04x} {:#04x} value {:#06x} index {:#06x}: the meter's protocol is on its \
             bulk endpoints",
            setup.request_type, setup.request, setup.value, setup.index
        ));
        Reply::Stall
    }

    fn out(&mut self, ep: u8, data: &[u8], v: &mut Violations) -> Reply {
        if ep != EP_OUT {
            v.report(format!("OUT transfer to endpoint {ep}, commands go to {EP_OUT}"));
            return Reply::Stall;
        }
        let Some(&command) = data.first() else {
            v.report("an empty command");
            return Reply::ack();
        };
        let i = usize::from(command);
        self.seen[i] = self.seen[i].wrapping_add(1);
        let action = self.fault(command, self.seen[i]);
        if action == Some(Action::StallOut) {
            return Reply::Stall;
        }
        let Some(mut answer) = self.answer(data, v) else {
            return Reply::ack();
        };
        let queued = match action {
            Some(Action::Silent) => return Reply::ack(),
            Some(Action::StallIn) => Answer::Stall,
            Some(Action::Short) => {
                answer.pop();
                Answer::Data(answer)
            }
            Some(Action::Garbage) => {
                answer[0] ^= 0x40;
                Answer::Data(answer)
            }
            Some(Action::StallOut) | None => Answer::Data(answer),
        };
        if self.answers.len() == QUEUE {
            v.report(format!("{QUEUE} answers unread, and one more command"));
            return Reply::ack();
        }
        self.answers.push_back(queued);
        Reply::ack()
    }

    fn in_transfer(&mut self, ep: u8, _max: usize, v: &mut Violations) -> Reply {
        if ep != EP_IN {
            v.report(format!("IN transfer from endpoint {ep}, answers come from {EP_IN}"));
            return Reply::Stall;
        }
        match self.answers.pop_front() {
            Some(Answer::Data(a)) => Reply::Data(a),
            Some(Answer::Stall) => Reply::Stall,
            None => Reply::Nak,
        }
    }

    fn reset(&mut self) {
        // The log and the clock are the meter's own; an answer is not.
        self.answers.clear();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn opts(list: &[(&str, &str)]) -> Vec<(String, String)> {
        list.iter().map(|(k, v)| ((*k).to_owned(), (*v).to_owned())).collect()
    }

    fn ask(dev: &mut dyn DeviceModel, data: &[u8], v: &mut Violations) -> Reply {
        assert_eq!(dev.out(EP_OUT, data, v), Reply::ack());
        dev.in_transfer(EP_IN, 64, v)
    }

    #[test]
    fn descriptors_are_the_wikis() {
        let dev = Kc330b::new();
        let d = dev.descriptors();
        assert_eq!((d.vendor_id(), d.product_id(), d.bcd_device()), (0x1041, 0x8101, 0x0100));
        assert_eq!(d.device[7], 64);
        assert_eq!(&d.device[14..17], &[0, 0, 0], "no strings");
        let config = d.config_by_value(1).unwrap();
        assert_eq!(config.len(), 32, "wTotalLength");
        assert_eq!(config[7], 0x80);
        assert_eq!(config[8], 10, "20 mA in 2 mA units");
        assert_eq!(Descriptors::interfaces(config), vec![[0, 0, 0xff, 1, 2]]);
        let eps = Descriptors::endpoints(config);
        assert_eq!(eps.len(), 2);
        assert_eq!((eps[0].address, eps[0].max_packet), (0x81, 64));
        assert_eq!((eps[1].address, eps[1].max_packet), (0x02, 64));
    }

    #[test]
    fn answers_what_the_driver_asks() {
        let mut dev = Kc330b::new();
        let mut v = Violations::default();
        assert_eq!(ask(&mut dev, &[2], &mut v), Reply::Data(b"\x82\x07KC-330B".to_vec()));
        assert_eq!(ask(&mut dev, &[4], &mut v), Reply::Data(vec![0x84]));
        assert_eq!(
            ask(&mut dev, &[5], &mut v),
            Reply::Data(vec![0x85, 2, 40, 120, 1, 1, 0, 0, 100])
        );
        assert_eq!(ask(&mut dev, &[6], &mut v), Reply::Data(vec![0x86, 13, 8, 3, 12, 0, 0]));
        assert_eq!(
            ask(&mut dev, &[7, 0, 0, 2], &mut v),
            Reply::Data(vec![0x87, 0x01, 0x2c, 0x01, 0x39])
        );
        assert_eq!(
            ask(&mut dev, &[7, 0, 1, 1], &mut v),
            Reply::Data({
                let b = stored_level(63).to_be_bytes();
                vec![0x87, b[0], b[1]]
            }),
            "the offset counts blocks of 63"
        );
        assert_eq!(ask(&mut dev, &[8], &mut v), Reply::Data(vec![0x88, 0x01, 0x2c]));
        assert_eq!(ask(&mut dev, &[8], &mut v), Reply::Data(vec![0x88, 0x01, 0x51]));
        assert_eq!(dev.in_transfer(EP_IN, 64, &mut v), Reply::Nak, "nothing asked");
        assert_eq!(ask(&mut dev, &[3, 26, 11, 4, 23, 59, 58], &mut v), Reply::Data(vec![0x83]));
        assert_eq!(dev.clock(), Some([26, 11, 4, 23, 59, 58]));
        assert_eq!(dev.stored(), 100, "the clock is not a configuration");
        assert_eq!(ask(&mut dev, &[1, 6, 40, 120, 1, 1, 0], &mut v), Reply::Data(vec![0x81]));
        assert_eq!(dev.config().map(|c| c.interval), Some(6));
        assert_eq!(dev.stored(), 0, "api.c:430");
        assert!(v.is_empty(), "{:?}", v.drain());
    }

    #[test]
    fn levels_stay_in_the_meters_range() {
        for n in 0..5000 {
            assert!((300..=1300).contains(&live_level(n)));
            assert!((300..=1300).contains(&stored_level(n as usize)));
        }
    }

    #[test]
    fn unread_answers_queue_in_order() {
        let mut dev = Kc330b::new();
        let mut v = Violations::default();
        assert_eq!(dev.out(EP_OUT, &[4], &mut v), Reply::ack());
        assert_eq!(dev.out(EP_OUT, &[8], &mut v), Reply::ack());
        assert_eq!(dev.in_transfer(EP_IN, 64, &mut v), Reply::Data(vec![0x84]));
        assert!(matches!(dev.in_transfer(EP_IN, 64, &mut v), Reply::Data(a) if a[0] == 0x88));
        for _ in 0..QUEUE {
            dev.out(EP_OUT, &[4], &mut v);
        }
        assert!(v.is_empty());
        dev.out(EP_OUT, &[4], &mut v);
        assert_eq!(v.drain().len(), 1, "queue full");
        dev.reset();
        assert_eq!(dev.in_transfer(EP_IN, 64, &mut v), Reply::Nak, "a reset drops them");
    }

    #[test]
    fn flags_what_the_protocol_forbids() {
        let mut dev = Kc330b::new();
        let mut v = Violations::default();
        let one = |name: &str, v: &mut Violations| assert_eq!(v.drain().len(), 1, "{name}");
        let s = Setup { request_type: 0x40, request: 1, value: 0, index: 0, length: 0 };
        assert_eq!(dev.control(s, &[], &mut v), Reply::Stall);
        one("a vendor request", &mut v);
        assert_eq!(dev.out(3, &[4], &mut v), Reply::Stall);
        one("OUT on another endpoint", &mut v);
        assert_eq!(dev.in_transfer(2, 64, &mut v), Reply::Stall);
        one("IN on another endpoint", &mut v);
        dev.out(EP_OUT, &[], &mut v);
        one("empty", &mut v);
        dev.out(EP_OUT, &[0x09], &mut v);
        one("unknown command", &mut v);
        for bad in [&[1u8][..], &[2, 0], &[3, 0], &[4, 0], &[5, 0], &[6, 0], &[7, 0], &[8, 0]] {
            dev.out(EP_OUT, bad, &mut v);
            one("wrong length", &mut v);
        }
        assert_eq!(dev.in_transfer(EP_IN, 64, &mut v), Reply::Nak, "none of them answered");
        ask(&mut dev, &[7, 0, 0, 0], &mut v);
        one("no samples", &mut v);
        ask(&mut dev, &[7, 0, 0, 64], &mut v);
        // 64 is both over a block and fits the log of 100.
        one("over a block", &mut v);
        ask(&mut dev, &[7, 0, 1, 63], &mut v);
        one("past the log", &mut v);
        let Reply::Data(cut) = ask(&mut dev, &[7, 0xff, 0xff, 63], &mut v) else { panic!() };
        one("far past the log", &mut v);
        assert_eq!(cut, vec![0x87]);
        for (i, bad) in [
            &[1u8, 7, 40, 120, 0, 0, 0],
            &[1, 0, 40, 120, 2, 0, 0],
            &[1, 0, 40, 120, 0, 2, 0],
            &[1, 0, 40, 120, 0, 0, 2],
        ]
        .iter()
        .enumerate()
        {
            ask(&mut dev, *bad, &mut v);
            assert_eq!(v.drain().len(), 1, "configure field {i}");
        }
        for bad in [
            [3u8, 26, 0, 4, 12, 0, 0],
            [3, 26, 13, 4, 12, 0, 0],
            [3, 26, 11, 0, 12, 0, 0],
            [3, 26, 11, 32, 12, 0, 0],
            [3, 26, 11, 4, 24, 0, 0],
            [3, 26, 11, 4, 12, 60, 0],
            [3, 26, 11, 4, 12, 0, 60],
        ] {
            ask(&mut dev, &bad, &mut v);
            assert_eq!(v.drain().len(), 1, "date {bad:?}");
        }
        assert!(v.is_empty());
    }

    #[test]
    fn faults_fire_as_asked() {
        let mut v = Violations::default();
        let mut dev = build(&opts(&[("stall-out", "08@2")])).unwrap();
        assert!(matches!(ask(dev.as_mut(), &[8], &mut v), Reply::Data(_)));
        assert_eq!(dev.out(EP_OUT, &[8], &mut v), Reply::Stall);
        assert_eq!(dev.in_transfer(EP_IN, 64, &mut v), Reply::Nak, "no effect");
        assert!(matches!(ask(dev.as_mut(), &[8], &mut v), Reply::Data(_)));

        let mut dev = build(&opts(&[("silent", "04")])).unwrap();
        assert_eq!(ask(dev.as_mut(), &[4], &mut v), Reply::Nak);
        assert_eq!(ask(dev.as_mut(), &[4], &mut v), Reply::Nak);

        let mut dev = build(&opts(&[("stall-in", "02@1")])).unwrap();
        assert_eq!(ask(dev.as_mut(), &[2], &mut v), Reply::Stall);
        assert!(matches!(ask(dev.as_mut(), &[2], &mut v), Reply::Data(_)));

        let mut dev = build(&opts(&[("short", "08"), ("short", "04")])).unwrap();
        assert_eq!(ask(dev.as_mut(), &[8], &mut v), Reply::Data(vec![0x88, 0x01]));
        assert_eq!(ask(dev.as_mut(), &[4], &mut v), Reply::Data(vec![]));

        let mut dev = build(&opts(&[("garbage", "01")])).unwrap();
        assert_eq!(ask(dev.as_mut(), &[1, 0, 40, 120, 0, 0, 0], &mut v), Reply::Data(vec![0xc1]));
        assert!(v.is_empty());
    }

    #[test]
    fn options_shape_the_answers() {
        let mut v = Violations::default();
        let mut dev = build(&opts(&[("inactive", "1")])).unwrap();
        assert_eq!(ask(dev.as_mut(), &[4], &mut v), Reply::Data(vec![0xa4]));
        for _ in 0..2 {
            assert_eq!(ask(dev.as_mut(), &[8], &mut v), Reply::Data(vec![0x88, 0x01, 0x2c]));
        }
        let mut dev = build(&opts(&[("model", "X"), ("id-length", "30")])).unwrap();
        assert_eq!(ask(dev.as_mut(), &[2], &mut v), Reply::Data(vec![0x82, 30, b'X']));
        let mut dev = build(&opts(&[
            ("stored", "300"),
            ("log-interval", "7"),
            ("log-time", "0"),
            ("log-freq", "0"),
            ("log-source", "17"),
        ]))
        .unwrap();
        assert_eq!(
            ask(dev.as_mut(), &[5], &mut v),
            Reply::Data(vec![0x85, 7, 40, 120, 0, 0, 17, 1, 44])
        );
        assert!(v.is_empty());
        assert_eq!(log_rules(&[]).digest_above, None);
        assert_eq!(log_rules(&opts(&[("digest", "1")])).digest_above, Some(DIGEST_ABOVE));
        assert_eq!(log_rules(&opts(&[("digest", "0")])).digest_above, None);
        for bad in [
            ("stored", "32001"),
            ("log-interval", "8"),
            ("log-time", "2"),
            ("inactive", "2"),
            ("silent", "8"),
            ("silent", "zz"),
            ("silent", "08@0"),
            ("model", "\u{e9}"),
            ("no-such", "1"),
        ] {
            assert!(build(&opts(&[bad])).is_err(), "{bad:?}");
        }
    }
}
