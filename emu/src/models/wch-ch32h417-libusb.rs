// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! The WCH CH32H417 logic analyzers (`1a86:5537`), the `WeAct` Studio
//! Logic Analyzer V2 among them, as the `wch-ch32h417-libusb` driver of
//! the vendor's sigrok fork sees them.
//!
//! Every fact below is taken from that C driver (`api.c`, `protocol.c`,
//! `protocol.h`, `libusb_wrapper.c` of `WeActStudio.U3LogicAnalyzer` at
//! `41b2564`, which `oxrok/libsigrok` carries). The firmware is closed and
//! no board or recording was at hand, so the model has never been held
//! against a real device. What the driver does not pin down is marked
//! "assumed" (the model's choice) or "invented" (made-up content).
//!
//! **The device**: ids `1a86:5537` (`USB_VENDOR_ID`, `USB_PRODUCT_ID`),
//! one configuration, interface 0 (the driver claims it) of the vendor
//! class (assumed), with bulk endpoints 1 OUT for commands, 1 IN for
//! answers, 2 IN for logic samples and 3 IN for ADC samples (`PIPE_CMD`,
//! `PIPE_LOGIC_DATA`, `PIPE_ADC_DATA`). The manufacturer and product
//! strings are invented; the driver reads neither. The serial number
//! string picks the board: the driver takes one that starts, in lower
//! case, with `b5` for the `WeAct` Studio Logic Analyzer V2 and with `01`
//! for the original board with its ADC, and no other
//! (`supported_devices[]`). `serial=TEXT` sets it; the default is a V2's
//! (the one in the driver's comment, `b5e339e33906e19c7f6416064c`).
//!
//! The real boards are USB 3 devices, and the emulator offers high speed
//! only. The driver tells the link by `bcdUSB`, below 3.00 being USB 2
//! (`ch32h417_libusb_detect_usb_speed()`), and shows other sample rates
//! for each. By default the model is the board on a USB 2 port, `bcdUSB`
//! 2.00 (assumed: what the firmware reports there is not in the driver).
//! `usb=3` reports 3.00 on the high speed link, an emulation artifact
//! that reaches the driver's USB 3 tables; the endpoints keep their 512
//! byte packets.
//!
//! **Commands** are 64 bytes on endpoint 1 (`PACKET_SIZE_OUT`):
//! `[command, length, data...]`, zero padded. Answers are 32 bytes
//! (`PACKET_SIZE_IN`, "fixed 32 bytes"): `[command | 0x10, length,
//! status, data...]`, the length counting the status and the data, zero
//! padded (assumed; the driver reads the declared length). The driver has
//! answer codes for these commands only, and reads none after a start or
//! a stop, so those have no answer (assumed):
//!
//! | command | data | answer's data |
//! |---|---|---|
//! | `a0` set logic parameters | width `08` or `10`, rate index, key stop | none |
//! | `a1` set threshold | 16 bits, high byte first | none |
//! | `a2` start logic | none | no answer |
//! | `a3` stop logic | none | no answer |
//! | `a4` get logic parameters | none | width, rate index, key count |
//! | `a5` get threshold | none | 1 byte (invented; no C path sends it) |
//! | `a6` set ADC parameters | width `08` or `0a`, rate index | none |
//! | `a7` set ADC channel | 0 or 1 | none |
//! | `a8` start ADC | none | no answer |
//! | `a9` stop ADC | none | no answer |
//! | `aa` get ADC parameters | none | width, rate index (no C path) |
//! | `ab` get ADC channel | none | 1 byte (no C path) |
//! | `ac` get version | none | firmware version, hardware version |
//! | `ae` enter the boot loader | none | flagged: no C path of sigrok-cli |
//! | `af` set LED brightness | percent | none |
//!
//! Firmware 0.2 counts presses of the board's key and reports the count
//! as the third byte of `a4`'s answer; firmware 0.1 answers two bytes and
//! reads two of `a0`'s three (`key=0`). `fw=N` and `hw=N` are the version
//! bytes (2 and 1 by default; the values are invented, the driver only
//! shows them).
//!
//! **The rate index** is the place of the rate in the table the driver
//! shows: 0 to 24 over USB 3, 0 to 5 over USB 2 with two byte samples and
//! 0 to 7 with one byte ones (the firmware adds an offset per USB mode,
//! says `protocol.c`); an ADC rate's index is 3 to 7. The model takes
//! those ranges as the device's rules.
//!
//! **The threshold**: a V2 takes the level at its input in tenths of a
//! volt, 10 to 33, with bit 15 set; the original board takes DAC counts
//! up to 1024 (`config_set()`).
//!
//! **A capture**: after `a2` the board streams on endpoint 2, a byte a
//! sample at width `08` and two bytes little endian at `10`, with no
//! framing; after `a8` on endpoint 3, a byte a sample at 8 bits and two at
//! 10. The model's samples are a counter from 0 at each start, so bit n
//! toggles every 2^n samples (ten bits of it in the ADC's two byte form).
//! The driver queues its 32 transfers of 256 KiB before the start; the
//! board answers nothing on a sample endpoint while it is idle.
//!
//! A real board streams until it is stopped, and stops on its own when
//! the link cannot carry the rate. As an emulation artifact the model
//! sends `transfers=N` transfers a capture (2 by default), each filled to
//! the host's length, and then falls silent, so that the number of
//! transfers a capture completes does not depend on when the stop lands.
//! A capture that wants more than they hold meets that silence, which is
//! the driver's path for a link that cannot keep up.
//!
//! `press=N` presses the board's key after the Nth sample transfer of a
//! capture: the count goes up by one and, when `a0` allowed the key to
//! end the capture, the stream ends there.
//!
//! **Pedantic checks**: a command that is not 64 bytes, is not zero
//! padded, declares more data than it has room for, is not one of the
//! table's, or has another data length than the table's (a `a0` of two
//! bytes is firmware 0.1's and is taken); a width, rate index, key flag,
//! threshold, brightness, ADC width or channel outside the above; an ADC
//! command to a board without one; a start with no parameters set since
//! the reset, or while a capture runs; the boot loader command; more than
//! 8 answers left unread; an answer read with fewer than 32 bytes; a
//! control request beyond the standard ones; a transfer to an endpoint
//! the board does not have.
//!
//! **Faults**, all off by default: `status=CC:NN` answers command `CC`
//! with status `NN`, `silent=CC` does not answer it, `short=CC` answers
//! with two bytes, `garbage=CC` answers as another command, `stall=CC`
//! STALLs the command itself, `stall-in=N` STALLs the Nth sample transfer
//! of each capture (a one-shot STALL is an emulation artifact: a real
//! one halts the endpoint until it is cleared), `unread=N` leaves N
//! answers nobody asked for on endpoint 1 after a reset, which the
//! driver's drain meets.

use crate::descriptor::{self, DeviceDescriptor};
use crate::model::{Descriptors, DeviceModel, Reply, Setup, Speed, Violations};
use crate::{Error, Result};
use std::collections::VecDeque;

/// The model name.
pub const NAME: &str = "wch-ch32h417-libusb";
/// What `oxrok-emu list` says about it.
pub const ABOUT: &str = "WCH CH32H417 logic analyzer, WeAct Studio Logic Analyzer V2 (1a86:5537, \
                         vendor bulk); serial=TEXT, usb=2|3, key=0|1, fw=N, hw=N, transfers=N, \
                         press=N; faults: status=CC:NN, silent=CC, short=CC, garbage=CC, stall=CC, \
                         stall-in=N, unread=N";

/// The ids of every board.
pub const VENDOR_ID: u16 = 0x1a86;
/// The product id.
pub const PRODUCT_ID: u16 = 0x5537;
/// The command endpoint, and the answer endpoint without its direction.
pub const EP_COMMAND: u8 = 1;
/// Logic samples.
pub const EP_LOGIC: u8 = 2;
/// ADC samples.
pub const EP_ADC: u8 = 3;
/// A command's length.
pub const COMMAND_SIZE: usize = 64;
/// An answer's length.
pub const ANSWER_SIZE: usize = 32;
/// A V2's serial number, from the C driver's comment.
pub const SERIAL_V2: &str = "b5e339e33906e19c7f6416064c";
/// Unread answers kept; one more is a violation.
pub const QUEUE: usize = 8;
/// Data over this is logged as its digest: the sample transfers.
pub const DIGEST_ABOVE: usize = 1024;
/// The bulk packet size at high speed.
const PACKET: u16 = 512;

const SET_LOGIC_PARAMS: u8 = 0xa0;
const SET_LOGIC_LEVEL: u8 = 0xa1;
const START_LOGIC: u8 = 0xa2;
const STOP_LOGIC: u8 = 0xa3;
const GET_LOGIC_PARAMS: u8 = 0xa4;
const GET_LOGIC_LEVEL: u8 = 0xa5;
const SET_ADC_PARAMS: u8 = 0xa6;
const SET_ADC_CHANNEL: u8 = 0xa7;
const START_ADC: u8 = 0xa8;
const STOP_ADC: u8 = 0xa9;
const GET_ADC_PARAMS: u8 = 0xaa;
const GET_ADC_CHANNEL: u8 = 0xab;
const GET_VERSION: u8 = 0xac;
const ENTER_IAP: u8 = 0xae;
const SET_LED_BRIGHT: u8 = 0xaf;

/// The samples of a logic transfer: a counter from `first`, a byte or two
/// little endian bytes each.
pub fn logic_samples(first: u64, bytes: usize, two_bytes: bool) -> Vec<u8> {
    if two_bytes {
        (0..bytes as u64 / 2).flat_map(|i| ((first + i) as u16).to_le_bytes()).collect()
    } else {
        (0..bytes as u64).map(|i| (first + i) as u8).collect()
    }
}

/// The samples of an ADC transfer: the same counter, ten bits of it in
/// the two byte form.
pub fn adc_samples(first: u64, bytes: usize, ten_bit: bool) -> Vec<u8> {
    if ten_bit {
        (0..bytes as u64 / 2).flat_map(|i| (((first + i) & 0x3ff) as u16).to_le_bytes()).collect()
    } else {
        (0..bytes as u64).map(|i| (first + i) as u8).collect()
    }
}

/// What a fault does to a command.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Fault {
    Status(u8),
    Silent,
    Short,
    Garbage,
    Stall,
}

/// A capture in progress.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Stream {
    /// The endpoint it runs on.
    endpoint: u8,
    /// Bytes a sample takes.
    two_bytes: bool,
    /// Transfers sent.
    sent: u32,
    /// Samples sent.
    samples: u64,
    /// The key ended it, or the model's budget did.
    ended: bool,
}

/// The emulated board.
#[allow(clippy::struct_excessive_bools)]
pub struct Ch32h417 {
    descriptors: Descriptors,
    /// The serial number makes it a V2 (`b5`).
    v2: bool,
    /// The serial number makes it the original board (`01`), with an ADC.
    adc: bool,
    usb3: bool,
    key: bool,
    version: [u8; 2],
    transfers: u32,
    press: Option<u32>,
    unread: usize,
    faults: Vec<(u8, Fault)>,
    stall_in: Option<u32>,
    answers: VecDeque<Vec<u8>>,
    /// The logic parameters last set: two byte samples, rate index, key
    /// stop.
    logic: Option<(bool, u8, bool)>,
    /// The ADC parameters last set: ten bits, rate index.
    adc_params: Option<(bool, u8)>,
    adc_channel: u8,
    threshold: Option<u16>,
    led: u8,
    key_seq: u8,
    stream: Option<Stream>,
}

/// The sample transfers are logged as digests.
pub fn log_rules(_options: &[(String, String)]) -> crate::txlog::Rules {
    crate::txlog::Rules { digest_above: Some(DIGEST_ABOVE), undefined: None }
}

fn hex_byte(key: &str, value: &str) -> Result<u8> {
    u8::from_str_radix(value, 16)
        .ok()
        .filter(|_| value.len() == 2)
        .ok_or_else(|| Error::Invalid(format!("{key}={value}: a command as two hex digits")))
}

/// A board from the `-o` options; see the module documentation.
///
/// # Errors
///
/// On an unknown option or a value out of range.
pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>> {
    let mut serial = SERIAL_V2.to_owned();
    let mut usb3 = false;
    let mut dev = Ch32h417::new(&serial, usb3);
    for (k, v) in options {
        let number = |max: u32| -> Result<u32> {
            v.parse()
                .ok()
                .filter(|n| *n <= max)
                .ok_or_else(|| Error::Invalid(format!("{k}={v}: 0 to {max}")))
        };
        match k.as_str() {
            "serial" => {
                if v.is_empty() || v.len() > 60 || !v.is_ascii() {
                    return Err(Error::Invalid(format!("serial={v}: ASCII, 1 to 60 bytes")));
                }
                v.clone_into(&mut serial);
            }
            "usb" => match v.as_str() {
                "2" => usb3 = false,
                "3" => usb3 = true,
                _ => return Err(Error::Invalid(format!("usb={v}: 2 or 3"))),
            },
            "key" => dev.key = number(1)? == 1,
            "fw" => dev.version[0] = number(255)? as u8,
            "hw" => dev.version[1] = number(255)? as u8,
            "transfers" => dev.transfers = number(1_000_000)?,
            "press" => {
                dev.press = Some(number(1_000_000)?)
                    .filter(|n| *n > 0)
                    .ok_or_else(|| Error::Invalid(format!("press={v}: from 1")))
                    .map(Some)?;
            }
            "unread" => dev.unread = number(64)? as usize,
            "status" => {
                let (c, s) = v
                    .split_once(':')
                    .ok_or_else(|| Error::Invalid(format!("status={v}: CC:NN, both hex")))?;
                dev.faults.push((hex_byte(k, c)?, Fault::Status(hex_byte(k, s)?)));
            }
            "silent" => dev.faults.push((hex_byte(k, v)?, Fault::Silent)),
            "short" => dev.faults.push((hex_byte(k, v)?, Fault::Short)),
            "garbage" => dev.faults.push((hex_byte(k, v)?, Fault::Garbage)),
            "stall" => dev.faults.push((hex_byte(k, v)?, Fault::Stall)),
            "stall-in" => {
                dev.stall_in = Some(number(1_000_000)?)
                    .filter(|n| *n > 0)
                    .ok_or_else(|| Error::Invalid(format!("stall-in={v}: from 1")))
                    .map(Some)?;
            }
            other => return Err(Error::Invalid(format!("{NAME} has no option {other}"))),
        }
    }
    dev.identify(&serial, usb3);
    dev.reset();
    Ok(Box::new(dev))
}

fn descriptors(serial: &str, usb3: bool) -> Descriptors {
    let mut body = Vec::new();
    body.extend_from_slice(&descriptor::interface(0, 0, 4, [0xff, 0, 0], 0));
    body.extend_from_slice(&descriptor::endpoint(EP_COMMAND, 2, PACKET, 0));
    body.extend_from_slice(&descriptor::endpoint(0x80 | EP_COMMAND, 2, PACKET, 0));
    body.extend_from_slice(&descriptor::endpoint(0x80 | EP_LOGIC, 2, PACKET, 0));
    body.extend_from_slice(&descriptor::endpoint(0x80 | EP_ADC, 2, PACKET, 0));
    let device = DeviceDescriptor {
        usb: if usb3 { 0x0300 } else { 0x0200 },
        class: [0, 0, 0],
        max_packet0: 64,
        vendor: VENDOR_ID,
        product: PRODUCT_ID,
        device: 0x0100,
        strings: [1, 2, 3],
        configurations: 1,
    };
    Descriptors {
        speed: Speed::High,
        device: device.bytes(),
        configs: vec![descriptor::configuration(1, 0, 0x80, 250, &body)],
        strings: vec!["oxrok-emu".into(), "CH32H417 Logic Analyzer".into(), serial.to_owned()],
    }
}

impl Default for Ch32h417 {
    fn default() -> Self {
        Self::new(SERIAL_V2, false)
    }
}

impl Ch32h417 {
    /// A board with this serial number, on a USB 2 link or claiming a
    /// USB 3 one, with firmware 0.2.
    pub fn new(serial: &str, usb3: bool) -> Ch32h417 {
        let mut dev = Ch32h417 {
            descriptors: descriptors(serial, usb3),
            v2: false,
            adc: false,
            usb3,
            key: true,
            version: [2, 1],
            transfers: 2,
            press: None,
            unread: 0,
            faults: Vec::new(),
            stall_in: None,
            answers: VecDeque::new(),
            logic: None,
            adc_params: None,
            adc_channel: 0,
            threshold: None,
            led: 20,
            key_seq: 0,
            stream: None,
        };
        dev.identify(serial, usb3);
        dev
    }

    fn identify(&mut self, serial: &str, usb3: bool) {
        let lower = serial.to_ascii_lowercase();
        self.v2 = lower.starts_with("b5");
        self.adc = lower.starts_with("01");
        self.usb3 = usb3;
        self.descriptors = descriptors(serial, usb3);
    }

    /// The logic parameters last set: two byte samples, rate index, key
    /// stop.
    pub fn logic_params(&self) -> Option<(bool, u8, bool)> {
        self.logic
    }

    /// The threshold last set, as sent.
    pub fn threshold(&self) -> Option<u16> {
        self.threshold
    }

    /// The indicator's brightness in percent.
    pub fn led(&self) -> u8 {
        self.led
    }

    /// Whether a capture runs.
    pub fn streaming(&self) -> bool {
        self.stream.is_some_and(|s| !s.ended)
    }

    /// The highest rate index of the link and the sample width.
    fn max_index(&self, two_bytes: bool) -> u8 {
        match (self.usb3, two_bytes) {
            (true, _) => 24,
            (false, true) => 5,
            (false, false) => 7,
        }
    }

    fn answer(command: u8, status: u8, data: &[u8]) -> Vec<u8> {
        let mut a = vec![command | 0x10, 1 + data.len() as u8, status];
        a.extend_from_slice(data);
        a.resize(ANSWER_SIZE, 0);
        a
    }

    fn start(&mut self, endpoint: u8, v: &mut Violations) {
        if self.streaming() {
            v.report("a start while a capture runs");
        }
        let two_bytes = if endpoint == EP_LOGIC {
            let Some((two_bytes, _, _)) = self.logic else {
                v.report("a logic start with no parameters set (a0) since the reset");
                return;
            };
            two_bytes
        } else {
            let Some((ten_bit, _)) = self.adc_params else {
                v.report("an ADC start with no parameters set (a6) since the reset");
                return;
            };
            ten_bit
        };
        self.stream = Some(Stream { endpoint, two_bytes, sent: 0, samples: 0, ended: false });
    }

    /// The data of a command's answer, or `None` for a command without
    /// one. Everything the board would not take is flagged.
    #[allow(clippy::too_many_lines)]
    fn execute(&mut self, command: u8, d: &[u8], v: &mut Violations) -> Option<Vec<u8>> {
        let lengths: &[usize] = match command {
            SET_LOGIC_PARAMS => &[3, 2],
            SET_LOGIC_LEVEL | SET_ADC_PARAMS => &[2],
            SET_ADC_CHANNEL | SET_LED_BRIGHT => &[1],
            START_LOGIC | STOP_LOGIC | GET_LOGIC_PARAMS | GET_LOGIC_LEVEL | START_ADC
            | STOP_ADC | GET_ADC_PARAMS | GET_ADC_CHANNEL | GET_VERSION | ENTER_IAP => &[0],
            other => {
                v.report(format!("command {other:#04x} is not one the board has"));
                return None;
            }
        };
        if !lengths.contains(&d.len()) {
            v.report(format!(
                "command {command:#04x} with {} data bytes, it takes {}",
                d.len(),
                lengths[0]
            ));
            return None;
        }
        let adc_command = matches!(
            command,
            SET_ADC_PARAMS | SET_ADC_CHANNEL | START_ADC | STOP_ADC | GET_ADC_PARAMS
                | GET_ADC_CHANNEL
        );
        if adc_command && !self.adc {
            v.report(format!("ADC command {command:#04x} to a board without an ADC"));
        }
        match command {
            SET_LOGIC_PARAMS => {
                let two_bytes = match d[0] {
                    0x08 => false,
                    0x10 => true,
                    other => {
                        v.report(format!("sample width {other:#04x}, not 0x08 or 0x10"));
                        other > 8
                    }
                };
                let max = self.max_index(two_bytes);
                if d[1] > max {
                    v.report(format!(
                        "rate index {} at width {:#04x} over USB {}, the table ends at {max}",
                        d[1],
                        d[0],
                        if self.usb3 { 3 } else { 2 }
                    ));
                }
                let key_stop = d.get(2).copied().unwrap_or(0);
                if key_stop > 1 {
                    v.report(format!("key stop flag {key_stop}, not 0 or 1"));
                }
                self.logic = Some((two_bytes, d[1], key_stop == 1 && self.key));
                Some(Vec::new())
            }
            SET_LOGIC_LEVEL => {
                let code = u16::from_be_bytes([d[0], d[1]]);
                let ok = if self.v2 {
                    code & 0x8000 != 0 && (10..=33).contains(&(code & 0x7fff))
                } else {
                    code <= 1024
                };
                if !ok {
                    v.report(format!(
                        "threshold {code:#06x}: {}",
                        if self.v2 {
                            "a V2 takes 10 to 33 tenths of a volt with bit 15 set"
                        } else {
                            "the DAC takes 0 to 1024"
                        }
                    ));
                }
                self.threshold = Some(code);
                Some(Vec::new())
            }
            START_LOGIC => {
                self.start(EP_LOGIC, v);
                None
            }
            START_ADC => {
                self.start(EP_ADC, v);
                None
            }
            // A stop of an idle board is the driver's habit before every
            // start, and ends a capture of either kind (assumed).
            STOP_LOGIC | STOP_ADC => {
                self.stream = None;
                None
            }
            GET_LOGIC_PARAMS => {
                let (two_bytes, index, _) = self.logic.unwrap_or((true, 0, false));
                let mut a = vec![if two_bytes { 0x10 } else { 0x08 }, index];
                if self.key {
                    a.push(self.key_seq);
                }
                Some(a)
            }
            GET_LOGIC_LEVEL => Some(vec![0]),
            SET_ADC_PARAMS => {
                let ten_bit = match d[0] {
                    0x08 => false,
                    0x0a => true,
                    other => {
                        v.report(format!("ADC width {other:#04x}, not 0x08 or 0x0a"));
                        other > 8
                    }
                };
                if !(3..=7).contains(&d[1]) {
                    v.report(format!("ADC rate index {}, the driver's are 3 to 7", d[1]));
                }
                self.adc_params = Some((ten_bit, d[1]));
                Some(Vec::new())
            }
            SET_ADC_CHANNEL => {
                if d[0] > 1 {
                    v.report(format!("ADC channel {}, not 0 or 1", d[0]));
                }
                self.adc_channel = d[0];
                Some(Vec::new())
            }
            GET_ADC_PARAMS => {
                let (ten_bit, index) = self.adc_params.unwrap_or((false, 3));
                Some(vec![if ten_bit { 0x0a } else { 0x08 }, index])
            }
            GET_ADC_CHANNEL => Some(vec![self.adc_channel]),
            GET_VERSION => Some(self.version.to_vec()),
            SET_LED_BRIGHT => {
                if d[0] > 100 {
                    v.report(format!("LED brightness {} percent, at most 100", d[0]));
                }
                self.led = d[0];
                Some(Vec::new())
            }
            _ => {
                v.report("the boot loader command (ae), which no capture or setting sends");
                None
            }
        }
    }

    fn samples_in(&mut self, ep: u8, max: usize) -> Reply {
        let Some(mut s) = self.stream.filter(|s| s.endpoint == ep && !s.ended) else {
            return Reply::Nak;
        };
        if s.sent >= self.transfers {
            return Reply::Nak;
        }
        s.sent += 1;
        let stalled = self.stall_in == Some(s.sent);
        let data = if ep == EP_LOGIC {
            logic_samples(s.samples, max, s.two_bytes)
        } else {
            adc_samples(s.samples, max, s.two_bytes)
        };
        s.samples += (data.len() / (1 + usize::from(s.two_bytes))) as u64;
        if self.press == Some(s.sent) {
            self.key_seq = self.key_seq.wrapping_add(1);
            s.ended = self.logic.is_some_and(|(_, _, key_stop)| key_stop);
        }
        self.stream = Some(s);
        if stalled { Reply::Stall } else { Reply::Data(data) }
    }
}

impl DeviceModel for Ch32h417 {
    fn name(&self) -> &'static str {
        NAME
    }

    fn descriptors(&self) -> &Descriptors {
        &self.descriptors
    }

    fn control(&mut self, setup: Setup, _data: &[u8], v: &mut Violations) -> Reply {
        v.report(format!(
            "request {:#04x} {:#04x} value {:#06x} index {:#06x}: the board's protocol is on its \
             bulk endpoints",
            setup.request_type, setup.request, setup.value, setup.index
        ));
        Reply::Stall
    }

    fn out(&mut self, ep: u8, data: &[u8], v: &mut Violations) -> Reply {
        if ep != EP_COMMAND {
            v.report(format!("OUT transfer to endpoint {ep}, commands go to {EP_COMMAND}"));
            return Reply::Stall;
        }
        if data.len() != COMMAND_SIZE {
            v.report(format!("a command of {} bytes, it is {COMMAND_SIZE}", data.len()));
            if data.len() < 2 {
                return Reply::ack();
            }
        }
        let (command, length) = (data[0], usize::from(data[1]));
        let Some(args) = data.get(2..2 + length) else {
            v.report(format!("command {command:#04x} declares {length} data bytes, more than it has"));
            return Reply::ack();
        };
        if data[2 + length..].iter().any(|b| *b != 0) {
            v.report(format!("command {command:#04x} is not zero padded"));
        }
        let fault = self.faults.iter().find(|(c, _)| *c == command).map(|(_, f)| *f);
        if fault == Some(Fault::Stall) {
            return Reply::Stall;
        }
        let Some(payload) = self.execute(command, args, v) else {
            return Reply::ack();
        };
        let answer = match fault {
            Some(Fault::Silent) => return Reply::ack(),
            Some(Fault::Status(s)) => Self::answer(command, s, &payload),
            Some(Fault::Short) => vec![command | 0x10, 1 + payload.len() as u8],
            Some(Fault::Garbage) => Self::answer(command ^ 0x40, 0, &payload),
            Some(Fault::Stall) | None => Self::answer(command, 0, &payload),
        };
        if self.answers.len() >= QUEUE {
            v.report(format!("{QUEUE} answers unread, and one more command"));
            return Reply::ack();
        }
        self.answers.push_back(answer);
        Reply::ack()
    }

    fn in_transfer(&mut self, ep: u8, max: usize, v: &mut Violations) -> Reply {
        match ep {
            EP_COMMAND => match self.answers.front() {
                Some(a) if max < a.len() => {
                    v.report(format!("an answer of {} bytes read with {max}", a.len()));
                    Reply::Data(self.answers.pop_front().unwrap_or_default())
                }
                Some(_) => Reply::Data(self.answers.pop_front().unwrap_or_default()),
                None => Reply::Nak,
            },
            EP_LOGIC | EP_ADC => self.samples_in(ep, max),
            _ => {
                v.report(format!("IN transfer from endpoint {ep}, which the board does not have"));
                Reply::Stall
            }
        }
    }

    fn reset(&mut self) {
        // A new attach is a board plugged in again: nothing is kept.
        self.answers = (0..self.unread).map(|_| Self::answer(GET_VERSION, 0, &self.version)).collect();
        self.logic = None;
        self.adc_params = None;
        self.adc_channel = 0;
        self.threshold = None;
        self.led = 20;
        self.key_seq = 0;
        self.stream = None;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn opts(list: &[(&str, &str)]) -> Vec<(String, String)> {
        list.iter().map(|(k, v)| ((*k).to_owned(), (*v).to_owned())).collect()
    }

    fn command(c: u8, data: &[u8]) -> Vec<u8> {
        let mut p = vec![c, data.len() as u8];
        p.extend_from_slice(data);
        p.resize(COMMAND_SIZE, 0);
        p
    }

    /// Sends a command and reads its answer as the driver does.
    fn ask(dev: &mut dyn DeviceModel, c: u8, data: &[u8], v: &mut Violations) -> Reply {
        assert_eq!(dev.out(EP_COMMAND, &command(c, data), v), Reply::ack());
        dev.in_transfer(EP_COMMAND, 512, v)
    }

    fn answered(r: Reply) -> Vec<u8> {
        match r {
            Reply::Data(d) => d,
            other => panic!("no answer: {other:?}"),
        }
    }

    fn violations(v: &mut Violations) -> Vec<String> {
        v.drain()
    }

    #[test]
    fn descriptors_are_the_drivers() {
        let dev = Ch32h417::default();
        let d = dev.descriptors();
        assert_eq!((d.vendor_id(), d.product_id()), (0x1a86, 0x5537));
        assert_eq!(u16::from_le_bytes([d.device[2], d.device[3]]), 0x0200);
        assert_eq!(d.strings[2], SERIAL_V2);
        let usb3 = build(&opts(&[("usb", "3")])).unwrap();
        let d = usb3.descriptors();
        assert_eq!(u16::from_le_bytes([d.device[2], d.device[3]]), 0x0300);
        assert!(build(&opts(&[("usb", "1")])).is_err());
        assert!(build(&opts(&[("serial", "")])).is_err());
        assert!(build(&opts(&[("status", "a0")])).is_err());
        assert!(build(&opts(&[("press", "0")])).is_err());
    }

    #[test]
    fn a_capture_as_the_driver_runs_it() {
        let mut dev = Ch32h417::default();
        let mut v = Violations::default();
        // The key count at the open.
        let a = answered(ask(&mut dev, GET_LOGIC_PARAMS, &[], &mut v));
        assert_eq!(a.len(), ANSWER_SIZE);
        assert_eq!(a[..6], [0xb4, 4, 0, 0x10, 0, 0]);
        // The stop before the start has no answer.
        assert_eq!(dev.out(EP_COMMAND, &command(STOP_LOGIC, &[]), &mut v), Reply::ack());
        assert_eq!(dev.in_transfer(EP_COMMAND, 512, &mut v), Reply::Nak);
        let a = answered(ask(&mut dev, SET_LOGIC_PARAMS, &[0x10, 5, 0], &mut v));
        assert_eq!(a[..3], [0xb0, 1, 0]);
        assert_eq!(dev.logic_params(), Some((true, 5, false)));
        // The transfers queued before the start get nothing.
        assert_eq!(dev.in_transfer(EP_LOGIC, 8, &mut v), Reply::Nak);
        assert_eq!(dev.out(EP_COMMAND, &command(START_LOGIC, &[]), &mut v), Reply::ack());
        assert!(dev.streaming());
        assert_eq!(dev.in_transfer(EP_LOGIC, 8, &mut v), Reply::Data(vec![0, 0, 1, 0, 2, 0, 3, 0]));
        assert_eq!(dev.in_transfer(EP_LOGIC, 4, &mut v), Reply::Data(vec![4, 0, 5, 0]));
        // Two transfers a capture, then silence.
        assert_eq!(dev.in_transfer(EP_LOGIC, 4, &mut v), Reply::Nak);
        assert_eq!(dev.out(EP_COMMAND, &command(STOP_LOGIC, &[]), &mut v), Reply::ack());
        assert!(!dev.streaming());
        // The next start counts from 0 again.
        assert_eq!(dev.out(EP_COMMAND, &command(START_LOGIC, &[]), &mut v), Reply::ack());
        assert_eq!(dev.in_transfer(EP_LOGIC, 2, &mut v), Reply::Data(vec![0, 0]));
        assert_eq!(violations(&mut v), Vec::<String>::new());
    }

    #[test]
    fn one_byte_samples_and_the_adc() {
        let mut dev = Ch32h417::new("0123", false);
        let mut v = Violations::default();
        answered(ask(&mut dev, SET_LOGIC_PARAMS, &[0x08, 7, 1], &mut v));
        dev.out(EP_COMMAND, &command(START_LOGIC, &[]), &mut v);
        assert_eq!(dev.in_transfer(EP_LOGIC, 3, &mut v), Reply::Data(vec![0, 1, 2]));
        dev.out(EP_COMMAND, &command(STOP_LOGIC, &[]), &mut v);
        answered(ask(&mut dev, SET_ADC_PARAMS, &[0x0a, 3], &mut v));
        answered(ask(&mut dev, SET_ADC_CHANNEL, &[1], &mut v));
        dev.out(EP_COMMAND, &command(START_ADC, &[]), &mut v);
        assert_eq!(dev.in_transfer(EP_LOGIC, 4, &mut v), Reply::Nak);
        assert_eq!(dev.in_transfer(EP_ADC, 4, &mut v), Reply::Data(vec![0, 0, 1, 0]));
        assert_eq!(violations(&mut v), Vec::<String>::new());
        assert_eq!(adc_samples(1022, 6, true), [0xfe, 0x03, 0xff, 0x03, 0, 0]);
        assert_eq!(logic_samples(254, 3, false), [254, 255, 0]);
        assert_eq!(logic_samples(65535, 4, true), [0xff, 0xff, 0, 0]);
    }

    #[test]
    fn settings_and_versions() {
        let mut dev = Ch32h417::default();
        let mut v = Violations::default();
        let a = answered(ask(&mut dev, GET_VERSION, &[], &mut v));
        assert_eq!(a[..5], [0xbc, 3, 0, 2, 1]);
        answered(ask(&mut dev, SET_LOGIC_LEVEL, &[0x80, 10], &mut v));
        assert_eq!(dev.threshold(), Some(0x800a));
        answered(ask(&mut dev, SET_LED_BRIGHT, &[100], &mut v));
        assert_eq!(dev.led(), 100);
        assert_eq!(violations(&mut v), Vec::<String>::new());
        // Firmware 0.1: no key count, and its two byte a0 is taken.
        let mut old = build(&opts(&[("key", "0"), ("fw", "1")])).unwrap();
        let a = answered(ask(old.as_mut(), GET_LOGIC_PARAMS, &[], &mut v));
        assert_eq!(a[..5], [0xb4, 3, 0, 0x10, 0]);
        answered(ask(old.as_mut(), SET_LOGIC_PARAMS, &[0x10, 0], &mut v));
        assert_eq!(answered(ask(old.as_mut(), GET_VERSION, &[], &mut v))[3], 1);
        assert_eq!(violations(&mut v), Vec::<String>::new());
    }

    /// Sends a command that must be flagged with `what`.
    fn flagged(dev: &mut Ch32h417, packet: &[u8], what: &str, v: &mut Violations) {
        dev.out(EP_COMMAND, packet, v);
        let got = v.drain();
        assert!(got.iter().any(|m| m.contains(what)), "{what}: {got:?}");
        dev.answers.clear();
    }

    /// One violation of each kind.
    #[test]
    fn what_the_board_would_not_take_is_flagged() {
        let mut v = Violations::default();
        let mut dev = Ch32h417::default();
        flagged(&mut dev, &command(SET_LED_BRIGHT, &[1])[..63], "a command of 63 bytes", &mut v);
        let mut padded = command(GET_VERSION, &[]);
        padded[40] = 1;
        flagged(&mut dev, &padded, "not zero padded", &mut v);
        let mut long = command(GET_VERSION, &[]);
        long[1] = 63;
        flagged(&mut dev, &long, "declares 63 data bytes", &mut v);
        flagged(&mut dev, &command(0x55, &[]), "not one the board has", &mut v);
        flagged(&mut dev, &command(SET_LED_BRIGHT, &[1, 2]), "with 2 data bytes", &mut v);
        flagged(&mut dev, &command(SET_LOGIC_PARAMS, &[0x09, 0, 0]), "sample width 0x09", &mut v);
        // Over USB 2 the two byte table has six rates and the other eight.
        flagged(&mut dev, &command(SET_LOGIC_PARAMS, &[0x10, 6, 0]), "the table ends at 5", &mut v);
        flagged(&mut dev, &command(SET_LOGIC_PARAMS, &[0x08, 8, 0]), "the table ends at 7", &mut v);
        flagged(&mut dev, &command(SET_LOGIC_PARAMS, &[0x10, 0, 2]), "key stop flag 2", &mut v);
        flagged(&mut dev, &command(SET_LOGIC_LEVEL, &[0x00, 20]), "threshold 0x0014", &mut v);
        flagged(&mut dev, &command(SET_LOGIC_LEVEL, &[0x80, 34]), "threshold 0x8022", &mut v);
        flagged(&mut dev, &command(SET_LED_BRIGHT, &[101]), "LED brightness 101", &mut v);
        flagged(&mut dev, &command(SET_ADC_CHANNEL, &[0]), "without an ADC", &mut v);
        flagged(&mut dev, &command(ENTER_IAP, &[]), "boot loader", &mut v);
        let mut fresh = Ch32h417::default();
        flagged(&mut fresh, &command(START_LOGIC, &[]), "no parameters set (a0)", &mut v);
        dev.out(EP_COMMAND, &command(SET_LOGIC_PARAMS, &[0x10, 0, 0]), &mut v);
        dev.out(EP_COMMAND, &command(START_LOGIC, &[]), &mut v);
        flagged(&mut dev, &command(START_LOGIC, &[]), "while a capture runs", &mut v);
        let mut usb3 = Ch32h417::new(SERIAL_V2, true);
        usb3.out(EP_COMMAND, &command(SET_LOGIC_PARAMS, &[0x10, 24, 0]), &mut v);
        assert!(v.is_empty());
        flagged(&mut usb3, &command(SET_LOGIC_PARAMS, &[0x10, 25, 0]), "the table ends at 24", &mut v);
        let mut adc = Ch32h417::new("0123", false);
        flagged(&mut adc, &command(SET_ADC_PARAMS, &[0x09, 3]), "ADC width 0x09", &mut v);
        flagged(&mut adc, &command(SET_ADC_PARAMS, &[0x08, 2]), "ADC rate index 2", &mut v);
        flagged(&mut adc, &command(SET_ADC_CHANNEL, &[2]), "ADC channel 2", &mut v);
        let mut idle = Ch32h417::new("0123", false);
        flagged(&mut idle, &command(START_ADC, &[]), "no parameters set (a6)", &mut v);
        flagged(&mut adc, &command(SET_LOGIC_LEVEL, &[0x80, 10]), "the DAC takes 0 to 1024", &mut v);
    }

    #[test]
    fn transfers_the_board_has_no_place_for_are_flagged() {
        let mut dev = Ch32h417::default();
        let mut v = Violations::default();
        assert_eq!(dev.out(2, &[0; 64], &mut v), Reply::Stall);
        assert!(v.drain()[0].contains("OUT transfer to endpoint 2"));
        assert_eq!(dev.in_transfer(4, 64, &mut v), Reply::Stall);
        assert!(v.drain()[0].contains("IN transfer from endpoint 4"));
        let setup = Setup { request_type: 0xc0, request: 1, value: 0, index: 0, length: 0 };
        assert_eq!(dev.control(setup, &[], &mut v), Reply::Stall);
        assert!(v.drain()[0].contains("bulk endpoints"));
        // An answer read with less than its 32 bytes.
        dev.out(EP_COMMAND, &command(GET_VERSION, &[]), &mut v);
        assert!(matches!(dev.in_transfer(EP_COMMAND, 16, &mut v), Reply::Data(_)));
        assert!(v.drain()[0].contains("read with 16"));
        // Nine commands with no answer read.
        for _ in 0..QUEUE {
            dev.out(EP_COMMAND, &command(GET_VERSION, &[]), &mut v);
        }
        assert!(v.is_empty());
        dev.out(EP_COMMAND, &command(GET_VERSION, &[]), &mut v);
        assert!(v.drain()[0].contains("answers unread"));
    }

    #[test]
    fn faults() {
        let mut v = Violations::default();
        let mut dev = build(&opts(&[("status", "a0:05")])).unwrap();
        assert_eq!(answered(ask(dev.as_mut(), SET_LOGIC_PARAMS, &[0x10, 0, 0], &mut v))[..3], [
            0xb0, 1, 5
        ]);
        let mut dev = build(&opts(&[("silent", "ac")])).unwrap();
        assert_eq!(ask(dev.as_mut(), GET_VERSION, &[], &mut v), Reply::Nak);
        let mut dev = build(&opts(&[("short", "ac")])).unwrap();
        assert_eq!(answered(ask(dev.as_mut(), GET_VERSION, &[], &mut v)), [0xbc, 3]);
        let mut dev = build(&opts(&[("garbage", "ac")])).unwrap();
        assert_eq!(answered(ask(dev.as_mut(), GET_VERSION, &[], &mut v))[0], 0xfc);
        let mut dev = build(&opts(&[("stall", "a0")])).unwrap();
        assert_eq!(
            dev.out(EP_COMMAND, &command(SET_LOGIC_PARAMS, &[0x10, 0, 0]), &mut v),
            Reply::Stall
        );
        // Answers nobody asked for, there after a reset.
        let mut dev = build(&opts(&[("unread", "2")])).unwrap();
        assert!(matches!(dev.in_transfer(EP_COMMAND, 512, &mut v), Reply::Data(_)));
        assert!(matches!(dev.in_transfer(EP_COMMAND, 512, &mut v), Reply::Data(_)));
        assert_eq!(dev.in_transfer(EP_COMMAND, 512, &mut v), Reply::Nak);
        // The second sample transfer of a capture stalls, once.
        let mut dev = build(&opts(&[("stall-in", "2"), ("transfers", "3")])).unwrap();
        answered(ask(dev.as_mut(), SET_LOGIC_PARAMS, &[0x10, 0, 0], &mut v));
        dev.out(EP_COMMAND, &command(START_LOGIC, &[]), &mut v);
        assert!(matches!(dev.in_transfer(EP_LOGIC, 4, &mut v), Reply::Data(_)));
        assert_eq!(dev.in_transfer(EP_LOGIC, 4, &mut v), Reply::Stall);
        assert!(matches!(dev.in_transfer(EP_LOGIC, 4, &mut v), Reply::Data(_)));
        assert_eq!(dev.in_transfer(EP_LOGIC, 4, &mut v), Reply::Nak);
        assert!(v.is_empty());
    }

    #[test]
    fn a_press_of_the_key_counts_and_ends_a_capture_that_allows_it() {
        let mut v = Violations::default();
        let key_seq = |dev: &mut dyn DeviceModel, v: &mut Violations| {
            answered(ask(dev, GET_LOGIC_PARAMS, &[], v))[5]
        };
        let mut dev = build(&opts(&[("press", "1"), ("transfers", "4")])).unwrap();
        answered(ask(dev.as_mut(), SET_LOGIC_PARAMS, &[0x10, 0, 1], &mut v));
        dev.out(EP_COMMAND, &command(START_LOGIC, &[]), &mut v);
        assert!(matches!(dev.in_transfer(EP_LOGIC, 4, &mut v), Reply::Data(_)));
        assert_eq!(dev.in_transfer(EP_LOGIC, 4, &mut v), Reply::Nak);
        assert_eq!(key_seq(dev.as_mut(), &mut v), 1);
        // Not allowed to end it: the count moves and the samples go on.
        dev.reset();
        answered(ask(dev.as_mut(), SET_LOGIC_PARAMS, &[0x10, 0, 0], &mut v));
        dev.out(EP_COMMAND, &command(START_LOGIC, &[]), &mut v);
        assert!(matches!(dev.in_transfer(EP_LOGIC, 4, &mut v), Reply::Data(_)));
        assert!(matches!(dev.in_transfer(EP_LOGIC, 4, &mut v), Reply::Data(_)));
        assert_eq!(key_seq(dev.as_mut(), &mut v), 1);
        assert!(v.is_empty());
    }

    #[test]
    fn a_reset_is_a_board_plugged_in_again() {
        let mut dev = Ch32h417::default();
        let mut v = Violations::default();
        answered(ask(&mut dev, SET_LOGIC_PARAMS, &[0x08, 1, 0], &mut v));
        dev.out(EP_COMMAND, &command(SET_LED_BRIGHT, &[50]), &mut v);
        dev.out(EP_COMMAND, &command(START_LOGIC, &[]), &mut v);
        dev.reset();
        assert!(!dev.streaming());
        assert_eq!((dev.logic_params(), dev.led()), (None, 20));
        assert_eq!(dev.in_transfer(EP_COMMAND, 512, &mut v), Reply::Nak);
    }
}
