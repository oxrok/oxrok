// Counterpart in sigrok:
//   libsigrok/src/hardware/testo/api.c (the scan, the transfers),
//   libsigrok/src/hardware/testo/protocol.c (the request and the answer's layout),
//   libsigrok/src/hardware/testo/protocol.h,
//   Copyright (C) 2014 Bert Vermeulen <bert@biot.com>,
//   with commits by Uwe Hermann, Lars-Peter Clausen, Soeren Apel,
//   Daniel Elstner, Martin Ling, Romain Tartière, Richard
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! A Testo 435 (435/635/735) as sigrok's `testo` driver reaches it: a
//! Renesas M16C behind an FT232R, which the driver talks to with plain
//! libusb, no libftdi.
//!
//! Sources, and how sure each fact is:
//!
//! - USB: the sigrok wiki's `lsusb -v` of a Testo 435-4
//!   (<https://sigrok.org/wiki/Testo_435-4/Info>, edited 2014): `128d:0003`,
//!   USB 2.00, bcdDevice 6.00 (the FT232R's), EP0 8 bytes, strings 1 `testo
//!   AG` and 2 `testo 435/635/735` (also iInterface), no serial, bus powered
//!   at 400 mA, bulk `0x81` and `0x02` of 64 bytes. Sure: a real device's.
//! - The chips: the wiki's page of the device (M30626FJPGP, FT232RQ). Sure.
//! - The request and the answer: `protocol.c`, and independently Testo's own
//!   description, `hhtcde.chm` in its `toolbox.exe`, linked from the wiki
//!   (pages `hhx35_e`, `hhx35r_e`, `hhx35ee_x`, `hhtcd7b_e`, `HHTCD4_e`,
//!   `exto_e`; read unpacked, not run). The request, 115200 8N1, `21 00 00 00
//!   01`, Size at offset 5, the count N of measurements, N times seven bytes
//!   (a little endian binary32, Unit, Unused, Decimal exponent) and a two
//!   byte check sequence. The CRC's algorithm is the C's. No capture exists,
//!   so what a device puts in **Size, Unused and Decimal exponent is unknown:
//!   they are 0 here, a stand-in**, and nothing reads them.
//! - Units 1 (temperature), 3 (humidity), 5 (wind speed), 24 (pressure): both
//!   sources. Testo's table lists about 170 others for all its instruments;
//!   the C refuses them. That a 435 sends one (set to °F, with a CO2 or a dew
//!   point reading) is an inference.
//! - Timing: Testo's example polls with `Sleep(200)` for up to 2000 ms and
//!   suggests a 7000 ms timeout for the 435/635/735. `delay=` is counted in
//!   reads, not milliseconds.
//!
//! The C's capture path has met no device since 2017: `b258c09f` dropped the
//! `sdi->driver` its scan needs, so stock libsigrok finds no Testo, and the
//! oracle here has that one line restored
//! (`tools/oracle-patches/libsigrok-testo-sdi-driver.patch`). What the C does
//! past its scan is weak evidence of the device.
//!
//! The driver's sequence, which [`Testo435`] holds it to: 4 vendor requests
//! set 115200 8N1, then each 9 byte request gets one answer. The driver raises
//! DTR and RTS, which the device does not need (Testo's example drives both
//! low), so neither level is a violation. It sends no latency timer, reset or
//! purge, where Testo's example asks for a 5 ms timer.
//!
//! **A C habit, accepted:** `FTDI_FLOW_NONE` is `0x0008` in `wValue`
//! (`protocol.h:41`), where the mode lives in `wIndex`'s high byte and
//! `wValue` holds XON/XOFF characters. The chip ignores it without XON/XOFF
//! and has no way to refuse, so the model hands the shell, whose check would
//! flag it, a `wValue` of 0 for exactly this request. Other values are
//! flagged.
//!
//! **Emulation artifacts**, all of them shared by the two tools:
//!
//! - An answer is ready at the first IN after its request, whole, where a
//!   real device needs milliseconds at 115200 Bd and the chip hands it over
//!   in pieces its latency timer decides. `delay=` and `chunk=` show those.
//! - The readings are not measured: the values step by a fixed amount per
//!   answer from a base per unit (or `values=`), counted from the bus reset.
//!
//! Options:
//!
//! - `units=A,B,...` the units measured, default `1,3`; an empty list is a
//!   device with no measurement. Repeating a unit (a thermocouple next to an
//!   air probe, both temperatures) is what `protocol.c:255-294` loops on.
//! - `units2=...` and `swap=N`: from answer N on (the first answer, the
//!   probe's, is 0) these units instead, as a probe plugged during a capture
//!   would, which is why the C has "Some channel hotswapped in!".
//! - `values=A,B,...` the readings, one per unit, instead of the steps
//!   (floats, `nan` and `inf` too).
//! - `manufacturer=`, `product=` replace the strings; empty removes one.
//! - `string-stall=1|2` STALLs reads of that string descriptor.
//! - `chunk=N` at most N bytes per IN transfer; `delay=N` N status-only reads
//!   before each answer.
//! - `tail=N`: N bytes of a previous answer's end arrive, in their own IN,
//!   right after the first request, as the C's "Tail end of some previous
//!   data" says. `stale=N`: N bytes wait in the chip from before, for the C's
//!   flush.
//! - `mute=N`: from answer N on, none. `mute-hold=1`: then NAK the reads
//!   instead of status-only packets. A real FT232R always answers within its
//!   latency timer, so `mute-hold` is an artifact, for a capture whose stop
//!   a case needs to find between transfers with a stable count.
//! - `bad-crc=N`, `bad-prefix=N`: answer N damaged, as line noise would.
//! - `out-stall=N`: the N-th write (from 0) and those after it STALL. A
//!   real chip would rather NAK; the driver sees an error either way.
//! - `stall-start=baud|data|flow-ctrl|modem-ctrl`: that request STALLs from
//!   its second time on, so the scan's goes through and the capture's fails.
//!   One-shot behaviour like this is an artifact.
//! - The FTDI shell's own: `stall=REQUEST[/WVALUE]`, `in-stall=N`, `idle=N`.
//!
//! Nothing here sends a zero length IN transfer: `protocol.c:95` would copy
//! `len - 2` bytes with a `len` of 0, but an FT232R always sends its status
//! bytes.

use crate::descriptor;
use crate::model::{Descriptors, DeviceModel, Reply, Setup, StandardFault, Violations};
use crate::shells::ftdi::{self, Chip, Faults, Fill, Ftdi, FtdiConfig, FtdiPorts, PortState};
use crate::{Error, Result};

/// The model name.
pub const NAME: &str = "testo";
/// What `oxrok-emu list` says about it.
pub const ABOUT: &str = "Testo 435/635/735 (128d:0003), an MCU behind an FT232R; units=, units2=, \
     swap=N, values=, manufacturer=, product=, string-stall=1|2, chunk=N, delay=N, tail=N, \
     stale=N, mute=N, mute-hold=1, bad-crc=N, bad-prefix=N, out-stall=N, \
     stall-start=baud|data|flow-ctrl|modem-ctrl; the FTDI shell's stall=, in-stall=, idle=";

/// The request, `TESTO_x35_REQUEST` (`api.c:40`).
pub const REQUEST: [u8; 9] = [0x12, 0, 0, 0, 1, 1, 0x55, 0xd1, 0xb7];

/// The most units an option may list. Past `MAX_REPLY_SIZE` (128 bytes,
/// 17 units) nothing new happens.
const MAX_UNITS: usize = 40;

/// The most bytes `tail=` and `stale=` take.
const MAX_TAIL: usize = 256;

/// The most bytes [`Mcu::pending`] holds. A host that asks and does not read
/// (a violation) loses what comes past it, as a full FIFO does.
const MAX_PENDING: usize = 1024;

/// CRC-16/MCRF4XX, as `crc16_mcrf4xx` in `protocol.c:189`, from `0xffff`.
pub fn crc(data: &[u8]) -> u16 {
    let mut crc = 0xffff_u16;
    for &b in data {
        crc ^= u16::from(b);
        for _ in 0..8 {
            crc = if crc & 1 != 0 { (crc >> 1) ^ 0x8408 } else { crc >> 1 };
        }
    }
    crc
}

/// The reading of `unit` in answer `n`: a value exact in binary32.
fn reading(unit: u8, n: u32) -> f32 {
    let (base, step) = match unit {
        1 => (21.5, 0.25),
        3 => (45.0, 0.5),
        5 => (1.25, 0.125),
        24 => (1013.25, 0.25),
        _ => (1.0, 1.0),
    };
    base + step * (n % 64) as f32
}

/// An answer: `values` are the readings, by unit.
pub fn answer(units: &[u8], values: &[f32]) -> Vec<u8> {
    let mut a = vec![0x21, 0, 0, 0, 1, 0, units.len() as u8];
    for (unit, value) in units.iter().zip(values) {
        a.extend_from_slice(&value.to_le_bytes());
        a.extend_from_slice(&[*unit, 0, 0]);
    }
    a.extend_from_slice(&crc(&a).to_le_bytes());
    a
}

/// What the device does, from the options.
#[derive(Clone, Debug)]
struct Behaviour {
    units: Vec<u8>,
    units2: Vec<u8>,
    swap: Option<u32>,
    values: Vec<f32>,
    string_stall: Vec<u8>,
    chunk: Option<usize>,
    delay: u32,
    tail: usize,
    stale: usize,
    mute: Option<u32>,
    mute_hold: bool,
    bad_crc: Option<u32>,
    bad_prefix: Option<u32>,
    out_stall: Option<u32>,
    stall_start: Option<u8>,
}

impl Default for Behaviour {
    fn default() -> Behaviour {
        Behaviour {
            units: vec![1, 3],
            units2: Vec::new(),
            swap: None,
            values: Vec::new(),
            string_stall: Vec::new(),
            chunk: None,
            delay: 0,
            tail: 0,
            stale: 0,
            mute: None,
            mute_hold: false,
            bad_crc: None,
            bad_prefix: None,
            out_stall: None,
            stall_start: None,
        }
    }
}

fn number<T: std::str::FromStr>(key: &str, value: &str) -> Result<T> {
    value.parse().map_err(|_| Error(format!("{key}={value}: not a number")))
}

fn units(key: &str, value: &str) -> Result<Vec<u8>> {
    let list: Vec<u8> =
        value.split(',').filter(|s| !s.is_empty()).map(|s| number(key, s)).collect::<Result<_>>()?;
    if list.len() > MAX_UNITS {
        return Err(Error(format!("{key}: at most {MAX_UNITS} units")));
    }
    Ok(list)
}

/// A device from the `-o` options.
///
/// # Errors
///
/// On an unknown option or a bad value.
pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>> {
    let mut config = config();
    let mut faults = Faults::default();
    let mut b = Behaviour::default();
    for (k, v) in options {
        match k.as_str() {
            "units" => b.units = units(k, v)?,
            "units2" => b.units2 = units(k, v)?,
            "swap" => b.swap = Some(number(k, v)?),
            "values" => {
                b.values = v.split(',').map(|s| number(k, s)).collect::<Result<Vec<f32>>>()?;
            }
            "manufacturer" => config.manufacturer = (!v.is_empty()).then(|| v.clone()),
            "product" => config.description = (!v.is_empty()).then(|| v.clone()),
            "string-stall" => b.string_stall.push(number(k, v)?),
            "chunk" => {
                let n: usize = number(k, v)?;
                if n == 0 {
                    return Err(Error("chunk=0: at least a byte".to_owned()));
                }
                b.chunk = Some(n);
            }
            "delay" => b.delay = number(k, v)?,
            "tail" => b.tail = number::<usize>(k, v)?.min(MAX_TAIL),
            "stale" => b.stale = number::<usize>(k, v)?.min(MAX_TAIL),
            "mute" => b.mute = Some(number(k, v)?),
            "mute-hold" => b.mute_hold = v == "1",
            "bad-crc" => b.bad_crc = Some(number(k, v)?),
            "bad-prefix" => b.bad_prefix = Some(number(k, v)?),
            "out-stall" => b.out_stall = Some(number(k, v)?),
            "stall-start" => {
                b.stall_start = Some(match v.as_str() {
                    "baud" => ftdi::request::SET_BAUD_RATE,
                    "data" => ftdi::request::SET_DATA,
                    "flow-ctrl" => ftdi::request::SET_FLOW_CTRL,
                    "modem-ctrl" => ftdi::request::MODEM_CTRL,
                    _ => return Err(Error(format!("stall-start={v}: one of the driver's four"))),
                });
            }
            other => {
                if !faults.option(other, v)? {
                    return Err(Error(format!("{NAME} has no option {other}")));
                }
            }
        }
    }
    let mut model = Testo435::new(config, b);
    model.chip.set_faults(faults);
    Ok(Box::new(model))
}

/// The board's descriptors, from the wiki's `lsusb -v`.
fn config() -> FtdiConfig {
    let mut c = FtdiConfig::new(Chip::R);
    c.vendor = 0x128d;
    c.product = 0x0003;
    c.manufacturer = Some("testo AG".to_owned());
    c.description = Some("testo 435/635/735".to_owned());
    c.serial = None;
    c.attributes = 0x80;
    c.max_power = 400;
    c
}

/// The MCU behind the chip's UART.
#[derive(Clone, Debug)]
struct Mcu {
    b: Behaviour,
    /// What the UART has for the host, at most [`MAX_PENDING`] bytes.
    pending: Vec<u8>,
    /// A piece that goes in its own IN transfer before `pending`.
    tail: Vec<u8>,
    /// Status-only reads left before `pending`.
    delay: u32,
    /// Answers given since the reset.
    answers: u32,
    /// Writes taken since the reset.
    writes: u32,
}

impl Mcu {
    fn new(b: Behaviour) -> Mcu {
        let pending = (0..b.stale).map(|i| i as u8 | 0x80).collect();
        Mcu { b, pending, tail: Vec::new(), delay: 0, answers: 0, writes: 0 }
    }

    /// Answer number `self.answers`, as the options have it.
    fn next_answer(&mut self) -> Vec<u8> {
        let n = self.answers;
        self.answers += 1;
        let units =
            if self.b.swap.is_some_and(|s| n >= s) { &self.b.units2 } else { &self.b.units };
        let values: Vec<f32> = units
            .iter()
            .enumerate()
            .map(|(i, &u)| self.b.values.get(i).copied().unwrap_or_else(|| reading(u, n)))
            .collect();
        let mut a = answer(units, &values);
        if self.b.bad_crc == Some(n) {
            let last = a.len() - 1;
            a[last] ^= 0xff;
        }
        if self.b.bad_prefix == Some(n) {
            a[0] ^= 0xff;
        }
        a
    }
}

impl FtdiPorts for Mcu {
    fn write(&mut self, _: usize, data: &[u8], state: &PortState, v: &mut Violations) -> Reply {
        let index = self.writes;
        self.writes += 1;
        if self.b.out_stall.is_some_and(|n| index >= n) {
            return Reply::Stall;
        }
        if state.baud().is_none_or(|b| b.abs_diff(115_200) > 115_200 * 3 / 100) {
            v.report(format!("request at {:?} Bd: the device talks 115200", state.baud()));
        }
        if state.line != 0x0008 {
            v.report(format!("request with SET_DATA {:#06x}: the device talks 8N1", state.line));
        }
        if state.flow != 0 {
            v.report(format!("request with flow control mode {}: the driver has none", state.flow));
        }
        if data != REQUEST {
            v.report(format!("{} bytes written that are not the request", data.len()));
            return Reply::ack();
        }
        if !self.pending.is_empty() || !self.tail.is_empty() {
            v.report(format!(
                "a request with {} bytes of the last answer unread",
                self.pending.len() + self.tail.len()
            ));
        }
        if index == 0 && self.b.tail > 0 {
            self.tail = (0..self.b.tail).map(|i| i as u8 | 0x80).collect();
        }
        if self.b.mute.is_some_and(|n| self.answers >= n) {
            return Reply::ack();
        }
        let a = self.next_answer();
        let room = MAX_PENDING.saturating_sub(self.pending.len());
        self.pending.extend_from_slice(&a[..a.len().min(room)]);
        self.delay = self.b.delay;
        Reply::ack()
    }

    fn read(&mut self, _: usize, max: usize, _: &PortState, _: &mut Violations) -> Fill {
        if !self.tail.is_empty() {
            let n = max.min(self.tail.len());
            return Fill::Data(self.tail.drain(..n).collect());
        }
        if self.pending.is_empty() {
            let muted = self.b.mute.is_some_and(|n| self.answers >= n);
            return if self.b.mute_hold && muted { Fill::Wait } else { Fill::Idle };
        }
        if self.delay > 0 {
            self.delay -= 1;
            return Fill::Idle;
        }
        let n = self.b.chunk.unwrap_or(max).min(max).min(self.pending.len());
        Fill::Data(self.pending.drain(..n).collect())
    }

    fn pins(&mut self, _: usize, _: &PortState, v: &mut Violations) -> u8 {
        v.report("READ_PINS: the driver has no use for the pins".to_owned());
        0
    }
}

/// The emulated device.
pub struct Testo435 {
    chip: Ftdi,
    mcu: Mcu,
    /// Times the `stall-start` request came.
    start_seen: u32,
}

impl Testo435 {
    fn new(config: FtdiConfig, b: Behaviour) -> Testo435 {
        Testo435 { chip: Ftdi::new(config), mcu: Mcu::new(b), start_seen: 0 }
    }
}

impl DeviceModel for Testo435 {
    fn name(&self) -> &'static str {
        NAME
    }

    fn descriptors(&self) -> &Descriptors {
        self.chip.descriptors()
    }

    fn control(&mut self, setup: Setup, data: &[u8], v: &mut Violations) -> Reply {
        if setup.request_type == 0x40 && self.mcu.b.stall_start == Some(setup.request) {
            self.start_seen += 1;
            if self.start_seen > 1 {
                return Reply::Stall;
            }
        }
        let mut setup = setup;
        // protocol.h:41, FTDI_FLOW_NONE: see the module documentation.
        if setup.request_type == 0x40
            && setup.request == ftdi::request::SET_FLOW_CTRL
            && setup.value == 0x0008
            && setup.index == 0
        {
            setup.value = 0;
        }
        self.chip.control(&mut self.mcu, setup, data, v)
    }

    fn out(&mut self, ep: u8, data: &[u8], v: &mut Violations) -> Reply {
        self.chip.out(&mut self.mcu, ep, data, v)
    }

    fn in_transfer(&mut self, ep: u8, max: usize, v: &mut Violations) -> Reply {
        self.chip.in_transfer(&mut self.mcu, ep, max, v)
    }

    fn reset(&mut self) {
        self.chip.reset();
        self.mcu = Mcu::new(self.mcu.b.clone());
        self.start_seen = 0;
    }

    fn fail_standard(&mut self, setup: Setup) -> Option<StandardFault> {
        const GET_DESCRIPTOR: u8 = 6;
        let string = setup.request_type == 0x80
            && setup.request == GET_DESCRIPTOR
            && setup.value >> 8 == u16::from(descriptor::STRING);
        (string && self.mcu.b.string_stall.contains(&((setup.value & 0xff) as u8)))
            .then_some(StandardFault::Stall)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn opts(list: &[(&str, &str)]) -> Vec<(String, String)> {
        list.iter().map(|(k, v)| ((*k).to_owned(), (*v).to_owned())).collect()
    }

    fn vendor(request: u8, value: u16) -> Setup {
        Setup { request_type: 0x40, request, value, index: 0, length: 0 }
    }

    /// `testo_set_serial_params`, in the C's order.
    fn serial(dev: &mut dyn DeviceModel, v: &mut Violations) {
        for (r, val) in [(3, 0x001a), (4, 0x0008), (2, 0x0008), (1, 0x0303)] {
            assert_eq!(dev.control(vendor(r, val), &[], v), Reply::ack());
        }
    }

    fn device(list: &[(&str, &str)]) -> (Box<dyn DeviceModel>, Violations) {
        let mut dev = build(&opts(list)).unwrap();
        let mut v = Violations::default();
        serial(dev.as_mut(), &mut v);
        (dev, v)
    }

    fn payload(r: Reply) -> Vec<u8> {
        let Reply::Data(d) = r else { panic!("{r:?}") };
        d[2..].to_vec()
    }

    #[test]
    fn descriptors_are_the_wikis() {
        let dev = build(&[]).unwrap();
        let d = dev.descriptors();
        assert_eq!(&d.device[8..12], &[0x8d, 0x12, 0x03, 0x00]);
        assert_eq!(&d.device[12..14], &[0x00, 0x06], "bcdDevice 6.00");
        assert_eq!(d.device[7], 8, "EP0");
        assert_eq!(&d.device[14..17], &[1, 2, 0]);
        assert_eq!(d.strings, ["testo AG", "testo 435/635/735"]);
        let c = &d.configs[0];
        assert_eq!((c[7], c[8]), (0x80, 200), "bus powered, 400 mA");
        assert_eq!(c[9 + 8], 2, "iInterface");
        assert_eq!(&c[18..25], &[7, 5, 0x81, 2, 64, 0, 0]);
        assert_eq!(&c[25..32], &[7, 5, 0x02, 2, 64, 0, 0]);
    }

    #[test]
    fn the_requests_crc_is_the_crc_of_the_answers() {
        assert_eq!(crc(&REQUEST[..7]).to_le_bytes(), [0xd1, 0xb7]);
    }

    #[test]
    fn a_request_gets_its_answer() {
        let (mut dev, mut v) = device(&[]);
        // Nothing yet: the status bytes alone.
        assert_eq!(dev.in_transfer(1, 128, &mut v), Reply::Data(vec![0x01, 0x60]));
        assert_eq!(dev.out(2, &REQUEST, &mut v), Reply::ack());
        let a = payload(dev.in_transfer(1, 128, &mut v));
        assert_eq!(a.len(), 7 + 2 * 7 + 2);
        assert_eq!(&a[..7], &[0x21, 0, 0, 0, 1, 0, 2]);
        assert_eq!(f32::from_le_bytes(a[7..11].try_into().unwrap()), 21.5);
        assert_eq!(a[11], 1);
        assert_eq!(f32::from_le_bytes(a[14..18].try_into().unwrap()), 45.0);
        assert_eq!(a[18], 3);
        assert_eq!(crc(&a[..a.len() - 2]).to_le_bytes(), a[a.len() - 2..]);
        // The next answer steps.
        dev.out(2, &REQUEST, &mut v);
        let a = payload(dev.in_transfer(1, 128, &mut v));
        assert_eq!(f32::from_le_bytes(a[7..11].try_into().unwrap()), 21.75);
        assert!(v.is_empty(), "{v:?}");
        // A reset starts over.
        dev.reset();
        serial(dev.as_mut(), &mut v);
        dev.out(2, &REQUEST, &mut v);
        let a = payload(dev.in_transfer(1, 128, &mut v));
        assert_eq!(f32::from_le_bytes(a[7..11].try_into().unwrap()), 21.5);
    }

    #[test]
    fn a_long_answer_has_status_bytes_in_each_packet() {
        let (mut dev, mut v) = device(&[("units", "1,3,5,24,1,3,5,24")]);
        dev.out(2, &REQUEST, &mut v);
        let Reply::Data(d) = dev.in_transfer(1, 128, &mut v) else { panic!() };
        assert_eq!(d.len(), 2 + 62 + 2 + 3);
        assert_eq!(&d[64..66], &[0x01, 0x60]);
    }

    #[test]
    fn flags_a_request_before_the_serial_settings() {
        let mut dev = build(&[]).unwrap();
        let mut v = Violations::default();
        dev.out(2, &REQUEST, &mut v);
        assert_eq!(v.drain().len(), 1, "no baud rate; DTR and RTS low are fine");
    }

    #[test]
    fn flags_wrong_serial_settings_one_by_one() {
        for (r, val, why) in [
            (3, 0x0034, "half the baud rate"),
            (4, 0x0007, "7 data bits"),
        ] {
            let (mut dev, mut v) = device(&[]);
            dev.control(vendor(r, val), &[], &mut v);
            assert!(v.is_empty(), "{why}: {v:?}");
            dev.out(2, &REQUEST, &mut v);
            assert_eq!(v.drain().len(), 1, "{why}");
        }
        let (mut dev, mut v) = device(&[]);
        let s = Setup { request_type: 0x40, request: 2, value: 0, index: 0x0100, length: 0 };
        dev.control(s, &[], &mut v);
        dev.out(2, &REQUEST, &mut v);
        assert_eq!(v.drain().len(), 1, "RTS/CTS");
    }

    #[test]
    fn dtr_and_rts_low_are_no_violation() {
        let (mut dev, mut v) = device(&[]);
        dev.control(vendor(1, 0x0300), &[], &mut v);
        dev.out(2, &REQUEST, &mut v);
        assert!(v.is_empty(), "{v:?}");
        assert_eq!(payload(dev.in_transfer(1, 128, &mut v))[0], 0x21);
    }

    #[test]
    fn what_is_not_read_stays_bounded() {
        let (mut dev, mut v) = device(&[("units", "1,1,1,1")]);
        for _ in 0..1000 {
            dev.out(2, &REQUEST, &mut v);
        }
        assert_eq!(v.drain().len(), 999, "a request over an unread answer");
        let mut got = 0;
        loop {
            let d = payload(dev.in_transfer(1, 64, &mut v));
            if d.is_empty() {
                break;
            }
            got += d.len();
        }
        assert_eq!(got, MAX_PENDING);
    }

    #[test]
    fn flags_a_flow_value_other_than_the_cs() {
        let (mut dev, mut v) = device(&[]);
        dev.control(vendor(2, 0x0009), &[], &mut v);
        assert_eq!(v.drain().len(), 1);
    }

    #[test]
    fn flags_a_wrong_request_and_gives_no_answer() {
        let (mut dev, mut v) = device(&[]);
        dev.out(2, &REQUEST[..8], &mut v);
        assert_eq!(v.drain().len(), 1);
        assert_eq!(dev.in_transfer(1, 128, &mut v), Reply::Data(vec![0x01, 0x60]));
    }

    #[test]
    fn flags_a_request_over_an_unread_answer() {
        let (mut dev, mut v) = device(&[]);
        dev.out(2, &REQUEST, &mut v);
        assert!(v.is_empty());
        dev.out(2, &REQUEST, &mut v);
        assert_eq!(v.drain().len(), 1);
    }

    #[test]
    fn flags_read_pins() {
        let mut dev = build(&[]).unwrap();
        let mut v = Violations::default();
        let s = Setup { request_type: 0xc0, request: 0x0c, value: 0, index: 0, length: 1 };
        dev.control(s, &[], &mut v);
        assert_eq!(v.drain().len(), 1);
    }

    #[test]
    fn chunk_and_delay_pace_by_reads() {
        let (mut dev, mut v) = device(&[("chunk", "5"), ("delay", "2"), ("units", "1")]);
        dev.out(2, &REQUEST, &mut v);
        assert_eq!(dev.in_transfer(1, 128, &mut v), Reply::Data(vec![0x01, 0x60]));
        assert_eq!(dev.in_transfer(1, 128, &mut v), Reply::Data(vec![0x01, 0x60]));
        assert_eq!(payload(dev.in_transfer(1, 128, &mut v)).len(), 5);
        assert_eq!(payload(dev.in_transfer(1, 128, &mut v)).len(), 5);
        assert_eq!(payload(dev.in_transfer(1, 128, &mut v)).len(), 5);
        assert_eq!(payload(dev.in_transfer(1, 128, &mut v)).len(), 1);
        assert_eq!(dev.in_transfer(1, 128, &mut v), Reply::Data(vec![0x01, 0x60]));
    }

    #[test]
    fn tail_comes_alone_before_the_first_answer_only() {
        let (mut dev, mut v) = device(&[("tail", "6"), ("units", "1")]);
        dev.out(2, &REQUEST, &mut v);
        assert_eq!(payload(dev.in_transfer(1, 128, &mut v)).len(), 6);
        assert_eq!(payload(dev.in_transfer(1, 128, &mut v))[0], 0x21);
        dev.out(2, &REQUEST, &mut v);
        assert_eq!(payload(dev.in_transfer(1, 128, &mut v))[0], 0x21);
    }

    #[test]
    fn stale_waits_from_before_any_request() {
        let (mut dev, mut v) = device(&[("stale", "4")]);
        assert_eq!(payload(dev.in_transfer(1, 128, &mut v)).len(), 4);
        assert_eq!(dev.in_transfer(1, 128, &mut v), Reply::Data(vec![0x01, 0x60]));
    }

    #[test]
    fn faults_damage_the_answer_they_name() {
        let (mut dev, mut v) = device(&[("bad-crc", "1"), ("bad-prefix", "2"), ("units", "1")]);
        let mut read = || {
            dev.out(2, &REQUEST, &mut v);
            payload(dev.in_transfer(1, 128, &mut v))
        };
        let a = read();
        assert_eq!(crc(&a[..a.len() - 2]).to_le_bytes(), a[a.len() - 2..]);
        let a = read();
        assert_ne!(crc(&a[..a.len() - 2]).to_le_bytes(), a[a.len() - 2..]);
        assert_ne!(read()[0], 0x21);
        assert_eq!(read()[0], 0x21);
    }

    #[test]
    fn mute_answers_nothing_from_then_on() {
        let (mut dev, mut v) = device(&[("mute", "1")]);
        dev.out(2, &REQUEST, &mut v);
        assert_eq!(payload(dev.in_transfer(1, 128, &mut v))[0], 0x21);
        dev.out(2, &REQUEST, &mut v);
        assert_eq!(dev.in_transfer(1, 128, &mut v), Reply::Data(vec![0x01, 0x60]));
        let (mut dev, mut v) = device(&[("mute", "0"), ("mute-hold", "1")]);
        dev.out(2, &REQUEST, &mut v);
        assert_eq!(dev.in_transfer(1, 128, &mut v), Reply::Nak);
    }

    #[test]
    fn swap_changes_the_units_at_the_answer_it_names() {
        let (mut dev, mut v) = device(&[("units", "1"), ("units2", "24,5"), ("swap", "1")]);
        dev.out(2, &REQUEST, &mut v);
        assert_eq!(payload(dev.in_transfer(1, 128, &mut v))[6], 1);
        dev.out(2, &REQUEST, &mut v);
        let a = payload(dev.in_transfer(1, 128, &mut v));
        assert_eq!((a[6], a[11], a[18]), (2, 24, 5));
    }

    #[test]
    fn values_replace_the_readings() {
        let (mut dev, mut v) = device(&[("units", "1"), ("values", "-12.5")]);
        dev.out(2, &REQUEST, &mut v);
        let a = payload(dev.in_transfer(1, 128, &mut v));
        assert_eq!(f32::from_le_bytes(a[7..11].try_into().unwrap()), -12.5);
    }

    #[test]
    fn stalls() {
        let (mut dev, mut v) = device(&[("out-stall", "1")]);
        assert_eq!(dev.out(2, &REQUEST, &mut v), Reply::ack());
        let mut first = vec![0x01, 0x60];
        first.extend(answer(&[1, 3], &[21.5, 45.0]));
        assert_eq!(dev.in_transfer(1, 128, &mut v), Reply::Data(first));
        assert_eq!(dev.out(2, &REQUEST, &mut v), Reply::Stall);
        // The scan's baud rate goes through, the start's does not.
        let mut dev = build(&opts(&[("stall-start", "baud")])).unwrap();
        assert_eq!(dev.control(vendor(3, 0x001a), &[], &mut v), Reply::ack());
        assert_eq!(dev.control(vendor(3, 0x001a), &[], &mut v), Reply::Stall);
        dev.reset();
        assert_eq!(dev.control(vendor(3, 0x001a), &[], &mut v), Reply::ack());
    }

    #[test]
    fn string_stall_names_one_string() {
        let mut dev = build(&opts(&[("string-stall", "1")])).unwrap();
        let s = |i: u16| Setup {
            request_type: 0x80,
            request: 6,
            value: 0x0300 | i,
            index: 0x0409,
            length: 255,
        };
        assert_eq!(dev.fail_standard(s(1)), Some(StandardFault::Stall));
        assert_eq!(dev.fail_standard(s(2)), None);
    }

    #[test]
    fn options_are_checked_and_stay_bounded() {
        for o in [
            ("chunk", "0"),
            ("speed", "1"),
            ("units", "x"),
            ("units", "256"),
            ("stall-start", "reset"),
            ("values", "a"),
        ] {
            assert!(build(&opts(&[o])).is_err(), "{o:?}");
        }
        let many = vec!["1"; MAX_UNITS + 1].join(",");
        assert!(build(&opts(&[("units", &many)])).is_err());
        let mut no_strings = build(&opts(&[("manufacturer", ""), ("product", "x")])).unwrap();
        assert_eq!(no_strings.descriptors().strings, ["x"]);
        no_strings.reset();
    }
}
