// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! A `GreatFET` One with its stock firmware, as libsigrok's `greatfet`
//! driver sees it.
//!
//! Where each fact comes from. The firmware sources read are
//! `greatscottgadgets/greatfet` at `da49f6b` (2026-08-21) and its
//! submodule `greatscottgadgets/libgreat` at `699d267`; no recording of
//! a real board was available, so everything below is that source and
//! the C driver's comments, and none of it is checked on hardware.
//!
//! - Descriptors, `firmware/greatfet_usb/usb_descriptor.c`: `1d50:60e6`,
//!   `bcdUSB` 2.00, class 0, EP0 of 64 bytes, `bcdDevice` 0x0100, strings
//!   `Great Scott Gadgets`, `GreatFET` and a serial number, one
//!   configuration of 500 mA with interface 0 (vendor, no endpoints, the
//!   libgreat command interface) and interface 1 (vendor, bulk IN 0x81
//!   and OUT 0x02, 512 bytes at high speed). String 3 is the 32 hex
//!   digits of the four words the LPC4330's IAP gives
//!   (`usb_set_descriptor_by_serial_number()`).
//! - Vendor request 0x65, `libgreat/firmware/platform/lpc43xx/drivers/usb/comms_backend.c`:
//!   an OUT of a prelude of two u32, class and verb, then the arguments,
//!   executes at once. `wIndex` bit 0 says no response will be read. An
//!   IN then reads the response; it gives what there is, up to `wLength`.
//!   Anything shorter than the prelude, an unknown class or verb, a
//!   verb's error or an IN with no command underway STALLs. A new OUT
//!   while a command waits for its IN STALLs too and clears the waiting
//!   command, which also holds for a command that failed: its failure
//!   leaves it underway when the host did not set bit 0.
//! - Core class 0, `libgreat/firmware/classes/core.c` and
//!   `greatfet/firmware/greatfet_usb/classes/core.c`: verb 1 the version
//!   string with its NUL (`comms_response_add_string()`), verb 3 the four
//!   IAP words of the serial number, little endian. The driver's comment
//!   (`protocol.c:362`) says local boards answer with two words of zero
//!   first; the model's serial number is a stand-in of that shape.
//! - Logic analyzer class 0x10d, `firmware/greatfet_usb/classes/logic_analyzer.c`:
//!   verbs numbered in table order, configure 0 (`<IB`, answers `<IIB`:
//!   rate, buffer size 0x4000, endpoint 0x81), first pin 1 (`<B`), alternate
//!   pin map 2 (`<?`), start 3, stop 4.
//! - The rate and the bus width, `libgreat/firmware/platform/lpc43xx/drivers/sgpio.c`:
//!   `sgpio_set_up_clocking()` divides the 204 MHz clock (the figure is
//!   the C driver's, `api.c:67`) by `clock / rate` rounded down and
//!   answers the rate that gives; rate 0 means undivided. A rate over the
//!   clock has no divider and comes back as asked.
//!   `sgpio_set_up_bus_topology()` makes 3 bits 4 and 5 to 7 bits 8, and
//!   refuses anything over 8. `verb_configure()` ignores
//!   `sgpio_set_up_functions()`'s result, so the host hears nothing of a
//!   refused setup.
//! - The stream, `firmware/greatfet_usb/usb_streaming.c`: start clears the
//!   endpoint's halt and streams full buffers of 0x4000 bytes, stop
//!   disables the endpoint. A host that does not keep up gets a halted
//!   endpoint and the stream ends (`streaming_detect_overrun()`).
//!
//! Inferred, not seen: after a setup the firmware refused (over 8 bits, a
//! rate over the clock), `sgpio_set_up_functions()` has returned before
//! generating the ISR that moves the samples, so the model sends no
//! samples and the host waits. `wide=1` stands for the firmware the C
//! driver's comment allows for (`protocol.c:69`), which would take more than
//! 8 channels: 16 bits a sample, two bytes, little endian;
//! none of the sources read has it. How samples sit in a byte is the C
//! driver's comment (`protocol.c:1246`): the first in the least
//! significant bits; the SGPIO's shift direction was not checked against it.
//!
//! The samples: sample `k` (`k` counts from 0 at each start and wraps at
//! [`PERIOD`]) has `k ^ (k >> 5) ^ (k >> 11)` on SGPIO0 to 7, and on SGPIO8
//! to 15 that reversed, xored with the high byte of `k`. A capture of the
//! upper pins so differs from the lower, and a transfer repeated or out
//! of place shows at any width, since 262144 bytes is no multiple of the
//! stream's period. `level=HHLL` holds the 16 pins at that hex value
//! instead, as a quiet input does, which makes every transfer one byte
//! repeated and a recording of one small.
//!
//! Emulation artifacts:
//!
//! - The real stream runs freely, so how many transfers complete before
//!   the host's stop depends on timing, and this driver resubmits a
//!   transfer before it looks at its limit. The model instead has
//!   `in-limit=N` transfers to give, one by default, then holds what is
//!   queued until the stop, so a case gets a fixed count as long as its
//!   sample limit falls in the last of them. A limit beyond them waits for
//!   ever, as with a stopped signal source.
//! - The stream is ready for the host at once; no time passes.
//! - `in-stall=N`'s halt answers one transfer, the others stay held until
//!   they are cancelled: a real halt fails them all.
//!
//! The pedantic checks hold the host to the protocol above and to what
//! `protocol.c` sends: request 0x65 and value 0 (the cancel value 0xdead
//! is pygreat's, not the driver's), recipient endpoint, `wIndex` bit 0
//! exactly on the commands with no response, an IN only for a waiting
//! command and long enough for its answer, classes and verbs the driver
//! uses, argument lengths, first pin 0 or 8 (the driver's comment,
//! `protocol.c:498`), the alternate pin map off, a bus width of 1 to 16,
//! start after configure, start and configure not during a capture, stop
//! only during one, samples read only after a start, whole packets, and
//! nothing on the bulk OUT endpoint.
//!
//! Faults, all off by default: `stall=VERB,...` STALLs the OUT of `serial`,
//! `version`, `first-pin`, `alt`, `configure`, `start` or `stop`, as a
//! firmware without that verb or a failed IAP does.
//! `stall-read=VERB,...` STALLs the IN of `serial`, `version` or
//! `configure`. `serial=HEX` and `version=HEX` answer those bytes
//! (`serial=` for an empty answer), for the text serial numbers
//! `protocol.c:352` says older firmware sends and for answers it refuses.
//! `configure-len=N` cuts configure's answer. `in-limit=N` above.
//! `in-stall=N` halts the endpoint after N transfers, as an overrun does.
//! `stall_string=3` and `short_string=3` fail the string read of `conn=sn=`.
//! `wide=1` and `level=HHLL` above.
//!
//! What the model taught about the C driver, from sigrok-cli's runs:
//!
//! - `libusb_control_transfer()` returns `LIBUSB_ERROR_BUSY` from inside
//!   a transfer callback, so the stop that `xfer_complete_cb()` reaches
//!   through `greatfet_abort_acquisition_quick()` and
//!   `greatfet_free_transfer()` never reaches the device, and the failure
//!   is logged once per transfer freed (15 frees and the abort): a halted
//!   endpoint or an error leaves the firmware's SGPIO running.
//! - `greatfet_free_transfer()` tests `!active_transfers` the wrong way
//!   (`protocol.c:961`): the end of the capture runs at the first transfer
//!   freed, not the last, and frees the feed queue unflushed, so an error
//!   loses what the queue holds (up to 4 MiB samples).
//! - The feed queue is only flushed when full or at the stop, so a client
//!   sees no sample for the first 16 transfers.
//! - `acq->start_req_sent = ret == SR_OK` (`api.c:524`) covers the
//!   submissions too: when they fail after a start that went through, no
//!   stop is ever sent.
//! - With a `probe_names` list starting with `-` and at most 8 names, the
//!   wire and the feed have one byte a sample, so
//!   `greatfet_process_receive_data()` takes its path for whole samples and passes
//!   packed bytes as if each were a sample (`protocol.c:1311`).
//!
//! Not reached in the driver, and why: a failing `libusb_open`; a device
//! without string 3 (`usb_descriptor.c` always has it); allocation
//! failures; `xfer_complete_cb()`'s timed out path (`protocol.c:872`
//! passes timeout 0), an unknown transfer status and a cancelled transfer
//! in `ACQ_RECEIVE` (cancelling comes after the state changed); and the
//! interface 0 release in `dev_close()`, as `control_interface_claimed` is
//! never set.

use std::fmt::Write as _;

use crate::descriptor::{self, DeviceDescriptor};
use crate::model::{Descriptors, DeviceModel, Reply, Setup, Speed, StandardFault, Violations};
use crate::{Error, Result};

/// The model name.
pub const NAME: &str = "greatfet";
/// What `oxrok-emu list` says about it.
pub const ABOUT: &str = "GreatFET One (1d50:60e6) with its stock firmware's logic analyzer; \
                         in-limit=N transfers to give (1), wide=1 for a firmware with 16 bit \
                         captures, level=HHLL for pins held still; faults: stall=serial,version,first-pin,alt,configure,start,\
                         stop, stall-read=serial,version,configure, serial=HEX, \
                         version=HEX, configure-len=N, in-stall=N, stall_string=I, short_string=I";

/// The libgreat command request.
const REQUEST: u8 = 0x65;
/// `LIBGREAT_REQUEST_FLAG_SKIP_RESPONSE`.
const FLAG_SKIP: u16 = 1;
/// The core class.
const CLASS_CORE: u32 = 0;
const VERB_VERSION: u32 = 1;
const VERB_SERIAL: u32 = 3;
/// The logic analyzer class.
const CLASS_LA: u32 = 0x10d;
const VERB_CONFIGURE: u32 = 0;
const VERB_FIRST_PIN: u32 = 1;
const VERB_ALT: u32 = 2;
const VERB_START: u32 = 3;
const VERB_STOP: u32 = 4;

/// `USB_STREAMING_BUFFER_SIZE`, which configure reports.
pub const BUFFER_SIZE: u32 = 0x4000;
/// `USB_STREAMING_IN_ADDRESS`.
pub const SAMPLES_ENDPOINT: u8 = 0x81;
/// The SGPIO clock, the figure in `api.c`'s comment on its rates.
pub const CLOCK: u32 = 204_000_000;
/// The firmware's version string, one the driver's comment names.
pub const VERSION: &[u8] = b"v2021.2.1\0";
/// The serial number's IAP words: a stand-in, of the shape
/// `protocol.c:362` describes.
pub const SERIAL_WORDS: [u32; 4] = [0, 0, 0x57cc_67e6, 0x3018_7657];
/// The samples repeat after this many; a multiple of 8, so that a whole
/// number of bytes holds them at any width.
pub const PERIOD: usize = 8 * 8191;
/// Bulk packets at high speed.
const PACKET: usize = 512;

/// A verb a fault can name.
fn verb_of(name: &str) -> Option<(u32, u32)> {
    Some(match name {
        "serial" => (CLASS_CORE, VERB_SERIAL),
        "version" => (CLASS_CORE, VERB_VERSION),
        "first-pin" => (CLASS_LA, VERB_FIRST_PIN),
        "alt" => (CLASS_LA, VERB_ALT),
        "configure" => (CLASS_LA, VERB_CONFIGURE),
        "start" => (CLASS_LA, VERB_START),
        "stop" => (CLASS_LA, VERB_STOP),
        _ => return None,
    })
}

/// The answer of the serial number verb.
fn serial_answer() -> Vec<u8> {
    SERIAL_WORDS.iter().flat_map(|w| w.to_le_bytes()).collect()
}

/// String 3: the words as `usb_set_descriptor_by_serial_number()` prints them.
pub fn serial_string() -> String {
    SERIAL_WORDS.iter().fold(String::new(), |mut text, w| {
        let _ = write!(text, "{w:08x}");
        text
    })
}

/// The descriptors of `usb_descriptor.c`.
pub fn firmware_descriptors() -> Descriptors {
    let vendor = [0xff, 0xff, 0xff];
    let mut body = Vec::new();
    body.extend_from_slice(&descriptor::interface(0, 0, 0, vendor, 0));
    body.extend_from_slice(&descriptor::interface(1, 0, 2, vendor, 0));
    body.extend_from_slice(&descriptor::endpoint(SAMPLES_ENDPOINT, crate::model::BULK, 512, 0));
    body.extend_from_slice(&descriptor::endpoint(0x02, crate::model::BULK, 512, 0));
    let config = descriptor::configuration(1, 0, 0x80, 500, &body);
    let dev = DeviceDescriptor {
        usb: 0x0200,
        class: [0, 0, 0],
        max_packet0: 64,
        vendor: 0x1d50,
        product: 0x60e6,
        device: 0x0100,
        strings: [1, 2, 3],
        configurations: 1,
    };
    Descriptors {
        speed: Speed::High,
        device: dev.bytes(),
        configs: vec![config],
        strings: vec!["Great Scott Gadgets".into(), "GreatFET".into(), serial_string()],
    }
}

/// The 16 pins at sample `k`.
pub fn pins(k: usize) -> u16 {
    let low = (k ^ (k >> 5) ^ (k >> 11)) as u8;
    let high = low.reverse_bits() ^ ((k >> 8) as u8);
    u16::from_le_bytes([low, high])
}

/// One period of the stream's bytes for a bus `width` bits wide (1, 2, 4
/// or 8) from `first_pin` (0 or 8).
pub fn packed(width: usize, first_pin: u8, level: Option<u16>) -> Vec<u8> {
    let per_byte = 8 / width;
    let mask = ((1u16 << width) - 1) as u8;
    let bank = |k: usize| (level.unwrap_or_else(|| pins(k)) >> first_pin) as u8;
    (0..PERIOD / per_byte)
        .map(|b| {
            (0..per_byte).fold(0u8, |acc, j| {
                acc | ((bank(b * per_byte + j) & mask) << (j * width))
            })
        })
        .collect()
}

/// What the model's capture sends, computed when the model is built: a
/// reply must not take long.
struct Streams {
    /// Per width index (1, 2, 4, 8 bits) and bank.
    narrow: [[Vec<u8>; 2]; 4],
    /// 16 bits a sample, for `wide=1`.
    wide: Vec<u8>,
}

impl Streams {
    fn new(level: Option<u16>) -> Streams {
        let bank = |w| [packed(w, 0, level), packed(w, 8, level)];
        Streams {
            narrow: [bank(1), bank(2), bank(4), bank(8)],
            wide: (0..PERIOD).flat_map(|k| level.unwrap_or_else(|| pins(k)).to_le_bytes()).collect(),
        }
    }

    fn get(&self, width: u8, first_pin: u8) -> &[u8] {
        if width == 16 {
            return &self.wide;
        }
        &self.narrow[width.trailing_zeros() as usize][usize::from(first_pin >= 8)]
    }
}

/// What configure set up.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Setup_ {
    /// The bus width after the firmware's rounding.
    width: u8,
    /// Whether `sgpio_set_up_functions()` went through.
    ok: bool,
}

/// Ways the board can misbehave; none by default.
#[derive(Clone, Debug, Default)]
struct Faults {
    stall: Vec<(u32, u32)>,
    stall_read: Vec<(u32, u32)>,
    serial: Option<Vec<u8>>,
    version: Option<Vec<u8>>,
    configure_len: Option<usize>,
    in_stall: Option<u32>,
    stall_string: Vec<u8>,
    short_string: Vec<u8>,
}

/// The emulated board.
#[allow(clippy::struct_excessive_bools)]
pub struct Greatfet {
    desc: Descriptors,
    streams: Streams,
    faults: Faults,
    in_limit: u32,
    wide: bool,
    /// The command waiting for its IN: class, verb, answer.
    underway: Option<(u32, u32, Vec<u8>)>,
    first_pin: u8,
    setup: Option<Setup_>,
    running: bool,
    started_once: bool,
    position: usize,
    completed: u32,
    halted: bool,
    /// A bulk IN was submitted before any start.
    early_submit: bool,
}

impl Greatfet {
    /// A board with these limits, faults aside.
    fn new(in_limit: u32, wide: bool, level: Option<u16>, faults: Faults) -> Greatfet {
        Greatfet {
            desc: firmware_descriptors(),
            streams: Streams::new(level),
            faults,
            in_limit,
            wide,
            underway: None,
            first_pin: 0,
            setup: None,
            running: false,
            started_once: false,
            position: 0,
            completed: 0,
            halted: false,
            early_submit: false,
        }
    }

    /// The rate `sgpio_set_up_clocking()` leaves, and whether it has a divider.
    fn clocking(rate: u32) -> (u32, bool) {
        if rate == 0 {
            return (CLOCK, true);
        }
        match CLOCK / rate {
            0 => (rate, false),
            divider => (CLOCK / divider, true),
        }
    }

    /// The bus `sgpio_set_up_bus_topology()` makes, if it can.
    fn topology(&self, width: u8) -> Option<u8> {
        match width {
            1 | 2 | 4 | 8 => Some(width),
            3 => Some(4),
            5..=7 => Some(8),
            9..=16 if self.wide => Some(16),
            _ => None,
        }
    }

    fn configure(&mut self, args: &[u8], v: &mut Violations) -> Option<Vec<u8>> {
        let [r0, r1, r2, r3, width] = *args else {
            v.report(format!("configure with {} bytes of arguments; it takes 5", args.len()));
            return None;
        };
        if self.running {
            v.report("configure during a capture; the driver stops first");
        }
        if width == 0 || width > 16 {
            v.report(format!("configure with {width} channels; the driver asks for 1 to 16"));
        }
        let (rate, clocked) = Self::clocking(u32::from_le_bytes([r0, r1, r2, r3]));
        let bus = self.topology(width);
        self.setup = Some(Setup_ { width: bus.unwrap_or(8), ok: clocked && bus.is_some() });
        let mut answer = Vec::with_capacity(9);
        answer.extend_from_slice(&rate.to_le_bytes());
        answer.extend_from_slice(&BUFFER_SIZE.to_le_bytes());
        answer.push(SAMPLES_ENDPOINT);
        if let Some(n) = self.faults.configure_len {
            answer.truncate(n);
        }
        Some(answer)
    }

    /// Executes a command; its answer, or `None` for a STALL.
    fn execute(&mut self, class: u32, verb: u32, args: &[u8], v: &mut Violations) -> Option<Vec<u8>> {
        if self.faults.stall.contains(&(class, verb)) {
            return None;
        }
        let no_args = |v: &mut Violations, name: &str| {
            if !args.is_empty() {
                v.report(format!("{name} with {} bytes of arguments; it takes none", args.len()));
            }
        };
        match (class, verb) {
            (CLASS_CORE, VERB_VERSION) => {
                no_args(v, "read version");
                Some(self.faults.version.clone().unwrap_or_else(|| VERSION.to_vec()))
            }
            (CLASS_CORE, VERB_SERIAL) => {
                no_args(v, "read serial number");
                Some(self.faults.serial.clone().unwrap_or_else(serial_answer))
            }
            (CLASS_LA, VERB_CONFIGURE) => self.configure(args, v),
            (CLASS_LA, VERB_FIRST_PIN) => {
                let [pin] = *args else {
                    v.report(format!("first pin with {} bytes of arguments; it takes 1", args.len()));
                    return None;
                };
                if pin > 15 {
                    // verb_change_first_pin(): EINVAL.
                    v.report(format!("first pin {pin}: the board has 16"));
                    return None;
                }
                if pin != 0 && pin != 8 {
                    v.report(format!("first pin {pin}: the driver uses 0 or 8"));
                }
                self.first_pin = pin;
                Some(Vec::new())
            }
            (CLASS_LA, VERB_ALT) => {
                let [alt] = *args else {
                    v.report(format!("alternate map with {} bytes of arguments; it takes 1", args.len()));
                    return None;
                };
                if alt != 0 {
                    v.report(format!("alternate map {alt}: the driver turns it off"));
                }
                Some(Vec::new())
            }
            (CLASS_LA, VERB_START) => {
                no_args(v, "start");
                if self.setup.is_none() {
                    v.report("start before configure");
                }
                if self.running {
                    v.report("start during a capture");
                }
                self.running = true;
                self.started_once = true;
                self.position = 0;
                self.completed = 0;
                self.halted = false;
                Some(Vec::new())
            }
            (CLASS_LA, VERB_STOP) => {
                no_args(v, "stop");
                if !self.running {
                    v.report("stop with no capture running");
                }
                self.running = false;
                Some(Vec::new())
            }
            _ => {
                // comms_backend_submit_command(): EINVAL for what the
                // firmware lacks; the verbs it has answer on a real board,
                // but the driver uses none of the others.
                v.report(format!("class {class:#x} verb {verb:#x}: the driver does not use it"));
                None
            }
        }
    }

    fn command_out(&mut self, setup: Setup, data: &[u8], v: &mut Violations) -> Reply {
        if setup.length as usize > 4096 {
            v.report(format!("command of {} bytes; the firmware takes 4096", setup.length));
            return Reply::Stall;
        }
        if data.len() < 8 {
            v.report(format!("command of {} bytes; the prelude has 8", data.len()));
            return Reply::Stall;
        }
        if self.underway.take().is_some() {
            // The firmware treats this as a host error: this STALLs and the
            // waiting command is dropped.
            v.report("a new command while the last one waits for its IN");
            return Reply::Stall;
        }
        let class = u32::from_le_bytes([data[0], data[1], data[2], data[3]]);
        let verb = u32::from_le_bytes([data[4], data[5], data[6], data[7]]);
        let wants_answer = matches!(
            (class, verb),
            (CLASS_CORE, VERB_VERSION | VERB_SERIAL) | (CLASS_LA, VERB_CONFIGURE)
        );
        let skip = setup.index & FLAG_SKIP != 0;
        if setup.index & !FLAG_SKIP != 0 {
            v.report(format!("command with wIndex {:#06x}; bit 0 is the only flag", setup.index));
        }
        if skip == wants_answer {
            v.report(format!(
                "class {class:#x} verb {verb:#x} with wIndex {:#06x}; the driver {} the answer",
                setup.index,
                if wants_answer { "reads" } else { "skips" }
            ));
        }
        let answer = self.execute(class, verb, &data[8..], v);
        let failed = answer.is_none();
        if !skip {
            // A command that failed stays underway too, as in the firmware.
            self.underway = Some((class, verb, answer.unwrap_or_default()));
        }
        if failed { Reply::Stall } else { Reply::ack() }
    }

    fn command_in(&mut self, setup: Setup, v: &mut Violations) -> Reply {
        if setup.index != 0 {
            v.report(format!("response read with wIndex {:#06x}; the driver sends 0", setup.index));
        }
        let Some((class, verb, answer)) = self.underway.take() else {
            v.report("response read with no command underway");
            return Reply::Stall;
        };
        if usize::from(setup.length) < answer.len() {
            v.report(format!(
                "response read of {} bytes; the answer has {}",
                setup.length,
                answer.len()
            ));
        }
        if self.faults.stall_read.contains(&(class, verb)) {
            return Reply::Stall;
        }
        Reply::Data(answer)
    }

    fn sample_bytes(&mut self, max: usize, stream_width: u8) -> Vec<u8> {
        let stream = self.streams.get(stream_width, self.first_pin);
        let mut data = Vec::with_capacity(max);
        let mut at = self.position % stream.len();
        while data.len() < max {
            let n = (stream.len() - at).min(max - data.len());
            data.extend_from_slice(&stream[at..at + n]);
            at = (at + n) % stream.len();
        }
        self.position = at;
        data
    }
}

/// `true` for "1", `false` for "0".
fn flag(k: &str, v: &str) -> Result<bool> {
    match v {
        "0" => Ok(false),
        "1" => Ok(true),
        _ => Err(Error(format!("{k}={v}: 0 or 1"))),
    }
}

fn hex(k: &str, v: &str) -> Result<Vec<u8>> {
    let bad = || Error(format!("{k}={v}: hex digits"));
    if !v.len().is_multiple_of(2) || !v.is_ascii() {
        return Err(bad());
    }
    (0..v.len() / 2).map(|i| u8::from_str_radix(&v[2 * i..2 * i + 2], 16).map_err(|_| bad())).collect()
}

/// A board from the `-o` options; see the module documentation.
///
/// # Errors
///
/// On an unknown option or value.
pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>> {
    let mut faults = Faults::default();
    let mut in_limit = 1;
    let mut wide = false;
    let mut level = None;
    let count = |k: &str, v: &str| -> Result<u32> {
        v.parse::<u32>().map_err(|_| Error(format!("{k}={v}: a count")))
    };
    let index = |k: &str, v: &str| -> Result<Vec<u8>> {
        v.split(',')
            .filter(|i| !i.is_empty())
            .map(|i| {
                i.parse::<u8>()
                    .ok()
                    .filter(|i| *i <= 3)
                    .ok_or_else(|| Error(format!("{k}={i}: a string index, 0 to 3")))
            })
            .collect()
    };
    for (k, v) in options {
        match k.as_str() {
            "in-limit" => in_limit = count(k, v)?,
            "wide" => wide = flag(k, v)?,
            "level" => {
                let bad = || Error(format!("level={v}: four hex digits"));
                level = Some(u16::from_str_radix(v, 16).map_err(|_| bad())?);
            }
            "stall" => {
                faults.stall.clear();
                for name in v.split(',').filter(|n| !n.is_empty()) {
                    faults.stall.push(
                        verb_of(name).ok_or_else(|| Error(format!("stall={name}: no such verb")))?,
                    );
                }
            }
            "stall-read" => {
                faults.stall_read.clear();
                for name in v.split(',').filter(|n| !n.is_empty()) {
                    faults.stall_read.push(
                        verb_of(name)
                            .ok_or_else(|| Error(format!("stall-read={name}: no such verb")))?,
                    );
                }
            }
            "serial" => faults.serial = Some(hex(k, v)?),
            "version" => faults.version = Some(hex(k, v)?),
            "configure-len" => faults.configure_len = Some(count(k, v)? as usize),
            "in-stall" => faults.in_stall = Some(count(k, v)?),
            "stall_string" => faults.stall_string = index(k, v)?,
            "short_string" => faults.short_string = index(k, v)?,
            other => return Err(Error(format!("{NAME} has no option {other}"))),
        }
    }
    Ok(Box::new(Greatfet::new(in_limit, wide, level, faults)))
}

impl DeviceModel for Greatfet {
    fn name(&self) -> &'static str {
        NAME
    }

    fn descriptors(&self) -> &Descriptors {
        &self.desc
    }

    fn control(&mut self, setup: Setup, data: &[u8], v: &mut Violations) -> Reply {
        if setup.request != REQUEST || setup.request_type & 0x7f != 0x42 {
            v.report(format!(
                "request {:#04x} {:#04x}: the driver sends {REQUEST:#04x} as vendor to the endpoint",
                setup.request_type, setup.request
            ));
            return Reply::Stall;
        }
        if setup.value != 0 {
            v.report(format!("command with wValue {:#06x}; the driver sends 0", setup.value));
            return Reply::Stall;
        }
        if setup.is_in() { self.command_in(setup, v) } else { self.command_out(setup, data, v) }
    }

    fn out(&mut self, ep: u8, _data: &[u8], v: &mut Violations) -> Reply {
        v.report(format!("OUT transfer to endpoint {ep}; the driver uses none"));
        Reply::Stall
    }

    fn in_transfer(&mut self, ep: u8, max: usize, v: &mut Violations) -> Reply {
        if ep != 1 {
            v.report(format!("IN transfer from endpoint {ep}; the samples come from 1"));
            return Reply::Stall;
        }
        if std::mem::take(&mut self.early_submit) {
            v.report("samples read before any start; the driver submits after it");
        }
        if !max.is_multiple_of(PACKET) {
            v.report(format!("bulk IN of {max} bytes; a transfer takes whole packets of {PACKET}"));
        }
        let Some(setup) = self.setup.filter(|s| self.running && s.ok && !self.halted) else {
            return Reply::Nak;
        };
        if self.faults.in_stall == Some(self.completed) {
            self.halted = true;
            return Reply::Stall;
        }
        if self.completed >= self.in_limit {
            return Reply::Nak;
        }
        self.completed += 1;
        Reply::Data(self.sample_bytes(max, setup.width))
    }

    fn in_submitted(&mut self, ep: u8, _max: usize) {
        if ep == 1 && !self.started_once {
            self.early_submit = true;
        }
    }

    fn reset(&mut self) {
        self.underway = None;
        self.first_pin = 0;
        self.setup = None;
        self.running = false;
        self.started_once = false;
        self.position = 0;
        self.completed = 0;
        self.halted = false;
        self.early_submit = false;
    }

    /// Transfers of 256 KiB.
    fn log_digest_above(&self) -> Option<usize> {
        Some(crate::txlog::HASH_ABOVE)
    }

    /// `stall_string` and `short_string`: `GET_DESCRIPTOR` of a string.
    fn fail_standard(&mut self, setup: Setup) -> Option<StandardFault> {
        const GET_DESCRIPTOR: u8 = 6;
        if setup.request_type != 0x80
            || setup.request != GET_DESCRIPTOR
            || setup.value >> 8 != u16::from(descriptor::STRING)
        {
            return None;
        }
        let index = (setup.value & 0xff) as u8;
        if self.faults.stall_string.contains(&index) {
            Some(StandardFault::Stall)
        } else if self.faults.short_string.contains(&index) {
            Some(StandardFault::Short(2))
        } else {
            None
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn opts(list: &[(&str, &str)]) -> Vec<(String, String)> {
        list.iter().map(|(k, v)| ((*k).to_string(), (*v).to_string())).collect()
    }

    fn board(list: &[(&str, &str)]) -> Box<dyn DeviceModel> {
        build(&opts(list)).unwrap()
    }

    fn out_setup(length: usize, index: u16) -> Setup {
        Setup { request_type: 0x42, request: REQUEST, value: 0, index, length: length as u16 }
    }

    fn in_setup(length: u16) -> Setup {
        Setup { request_type: 0xc2, request: REQUEST, value: 0, index: 0, length }
    }

    fn command(class: u32, verb: u32, args: &[u8]) -> Vec<u8> {
        let mut c = class.to_le_bytes().to_vec();
        c.extend_from_slice(&verb.to_le_bytes());
        c.extend_from_slice(args);
        c
    }

    /// A command with no answer, as `greatfet_trivial_request()` sends it.
    fn quiet(m: &mut dyn DeviceModel, class: u32, verb: u32, args: &[u8], v: &mut Violations) -> Reply {
        let c = command(class, verb, args);
        m.control(out_setup(c.len(), 1), &c, v)
    }

    fn ask(m: &mut dyn DeviceModel, class: u32, verb: u32, args: &[u8], v: &mut Violations) -> Reply {
        let c = command(class, verb, args);
        assert_eq!(m.control(out_setup(c.len(), 0), &c, v), Reply::ack());
        m.control(in_setup(1024), &[], v)
    }

    fn configure(m: &mut dyn DeviceModel, rate: u32, width: u8, v: &mut Violations) -> Vec<u8> {
        let mut args = rate.to_le_bytes().to_vec();
        args.push(width);
        let Reply::Data(d) = ask(m, CLASS_LA, VERB_CONFIGURE, &args, v) else { panic!("no answer") };
        d
    }

    fn fires(m: &mut dyn DeviceModel, f: impl FnOnce(&mut dyn DeviceModel, &mut Violations), what: &str) {
        let mut v = Violations::default();
        f(m, &mut v);
        let list = v.drain();
        assert!(list.iter().any(|l| l.contains(what)), "{what} not in {list:?}");
    }

    #[test]
    fn answers_the_driver_at_scan_and_capture_without_a_violation() {
        let mut m = board(&[]);
        let mut v = Violations::default();
        assert_eq!(ask(&mut *m, CLASS_CORE, VERB_SERIAL, &[], &mut v), Reply::Data(serial_answer()));
        assert_eq!(ask(&mut *m, CLASS_CORE, VERB_VERSION, &[], &mut v), Reply::Data(VERSION.to_vec()));
        assert_eq!(quiet(&mut *m, CLASS_LA, VERB_FIRST_PIN, &[0], &mut v), Reply::ack());
        assert_eq!(quiet(&mut *m, CLASS_LA, VERB_ALT, &[0], &mut v), Reply::ack());
        let answer = configure(&mut *m, 34_000_000, 8, &mut v);
        assert_eq!(answer[..4], 34_000_000u32.to_le_bytes());
        assert_eq!(answer[4..8], 0x4000u32.to_le_bytes());
        assert_eq!(answer[8], 0x81);
        assert_eq!(quiet(&mut *m, CLASS_LA, VERB_START, &[], &mut v), Reply::ack());
        m.in_submitted(1, 262_144);
        let Reply::Data(d) = m.in_transfer(1, 262_144, &mut v) else { panic!("no samples") };
        assert_eq!(d.len(), 262_144);
        assert_eq!(d[..4], packed(8, 0, None)[..4]);
        // in-limit=1: the rest is held until the stop.
        assert_eq!(m.in_transfer(1, 262_144, &mut v), Reply::Nak);
        assert_eq!(quiet(&mut *m, CLASS_LA, VERB_STOP, &[], &mut v), Reply::ack());
        assert_eq!(m.in_transfer(1, 262_144, &mut v), Reply::Nak);
        assert!(v.is_empty(), "{:?}", v.drain());
    }

    #[test]
    fn rates_follow_the_sgpio_divider() {
        assert_eq!(Greatfet::clocking(34_000_000), (34_000_000, true));
        assert_eq!(Greatfet::clocking(30_000_000), (34_000_000, true));
        assert_eq!(Greatfet::clocking(72_000_000), (102_000_000, true));
        assert_eq!(Greatfet::clocking(10_000_000), (10_200_000, true));
        assert_eq!(Greatfet::clocking(0), (CLOCK, true));
        assert_eq!(Greatfet::clocking(CLOCK), (CLOCK, true));
        assert_eq!(Greatfet::clocking(CLOCK + 1), (CLOCK + 1, false));
    }

    #[test]
    fn a_refused_setup_answers_and_then_sends_nothing() {
        for (rate, width) in [(1_000_000, 9), (CLOCK + 1, 1)] {
            let mut m = board(&[]);
            let mut v = Violations::default();
            let answer = configure(&mut *m, rate, width, &mut v);
            assert_eq!(answer.len(), 9);
            quiet(&mut *m, CLASS_LA, VERB_START, &[], &mut v);
            assert_eq!(m.in_transfer(1, 512, &mut v), Reply::Nak);
            assert!(v.is_empty());
        }
    }

    #[test]
    fn wide_sends_two_bytes_a_sample() {
        let mut m = board(&[("wide", "1")]);
        let mut v = Violations::default();
        configure(&mut *m, 20_400_000, 16, &mut v);
        quiet(&mut *m, CLASS_LA, VERB_START, &[], &mut v);
        let Reply::Data(d) = m.in_transfer(1, 512, &mut v) else { panic!("no samples") };
        assert_eq!(d[2..4], pins(1).to_le_bytes());
    }

    #[test]
    fn packed_samples_are_first_in_the_low_bits() {
        let p = packed(2, 0);
        let expected = (0..4).fold(0u8, |a, j| a | ((pins(j) as u8 & 3) << (2 * j)));
        assert_eq!(p[0], expected);
        assert_eq!(packed(1, 8, None)[0] & 1, (pins(0) >> 8) as u8 & 1);
        assert_eq!(packed(8, 8, None)[1], (pins(1) >> 8) as u8);
    }

    /// One period's worth of a transfer's bytes, and the next transfer's.
    fn two_transfers(m: &mut dyn DeviceModel, width: u8) -> (Vec<u8>, Vec<u8>) {
        let mut v = Violations::default();
        configure(m, 1_000_000, width, &mut v);
        quiet(m, CLASS_LA, VERB_START, &[], &mut v);
        let mut read = || {
            let Reply::Data(d) = m.in_transfer(1, 262_144, &mut v) else { panic!("no samples") };
            d
        };
        (read(), read())
    }

    #[test]
    fn a_transfer_differs_from_the_next_at_any_width() {
        for width in [1, 2, 4, 8] {
            let (a, b) = two_transfers(&mut *board(&[("in-limit", "2")]), width);
            assert_ne!(a, b, "width {width}");
            assert!(a.iter().any(|x| *x != a[0]), "width {width}");
        }
    }

    #[test]
    fn a_level_holds_the_pins() {
        let m = &mut *board(&[("in-limit", "2"), ("level", "a5c3")]);
        let (a, b) = two_transfers(m, 8);
        assert_eq!(a, b);
        assert_eq!((a[0], a[261_000]), (0xc3, 0xc3));
        assert_eq!(packed(2, 0, Some(0xa5c3))[0], 0xff);
        assert_eq!(packed(4, 8, Some(0xa5c3))[0], 0x55);
        assert!(build(&opts(&[("level", "xyz")])).is_err());
    }

    #[test]
    fn widths_round_as_the_firmware_does() {
        let m = Greatfet::new(1, false, None, Faults::default());
        let got: Vec<_> = (1..=9).map(|w| m.topology(w)).collect();
        let want = [Some(1), Some(2), Some(4), Some(4), Some(8), Some(8), Some(8), Some(8), None];
        assert_eq!(got, want);
    }

    /// Drives the model into one violation.
    type Check = fn(&mut dyn DeviceModel, &mut Violations);

    #[test]
    #[allow(clippy::too_many_lines)]
    fn each_check_fires() {
        let cases: Vec<(&str, Check)> = vec![
            ("as vendor to the endpoint", |m, v| {
                let s = Setup { request_type: 0x40, request: REQUEST, value: 0, index: 0, length: 0 };
                assert_eq!(m.control(s, &[], v), Reply::Stall);
            }),
            ("as vendor to the endpoint", |m, v| {
                let s = Setup { request_type: 0x42, request: 0x66, value: 0, index: 0, length: 0 };
                assert_eq!(m.control(s, &[], v), Reply::Stall);
            }),
            ("wValue", |m, v| {
                let s = Setup { request_type: 0xc2, request: REQUEST, value: 0xdead, index: 0, length: 4 };
                assert_eq!(m.control(s, &[], v), Reply::Stall);
            }),
            ("firmware takes 4096", |m, v| {
                assert_eq!(m.control(out_setup(4097, 0), &[], v), Reply::Stall);
            }),
            ("the prelude has 8", |m, v| {
                assert_eq!(m.control(out_setup(4, 0), &[0; 4], v), Reply::Stall);
            }),
            ("bit 0 is the only flag", |m, v| {
                let c = command(CLASS_LA, VERB_START, &[]);
                m.control(out_setup(c.len(), 3), &c, v);
            }),
            ("the driver reads the answer", |m, v| {
                quiet(m, CLASS_CORE, VERB_SERIAL, &[], v);
            }),
            ("the driver skips the answer", |m, v| {
                let c = command(CLASS_LA, VERB_ALT, &[0]);
                m.control(out_setup(c.len(), 0), &c, v);
            }),
            ("waits for its IN", |m, v| {
                let c = command(CLASS_CORE, VERB_SERIAL, &[]);
                m.control(out_setup(c.len(), 0), &c, v);
                assert_eq!(m.control(out_setup(c.len(), 0), &c, v), Reply::Stall);
            }),
            ("no command underway", |m, v| {
                assert_eq!(m.control(in_setup(9), &[], v), Reply::Stall);
            }),
            ("response read with wIndex", |m, v| {
                let c = command(CLASS_CORE, VERB_SERIAL, &[]);
                m.control(out_setup(c.len(), 0), &c, v);
                let s = Setup { request_type: 0xc2, request: REQUEST, value: 0, index: 1, length: 16 };
                m.control(s, &[], v);
            }),
            ("the answer has 16", |m, v| {
                let c = command(CLASS_CORE, VERB_SERIAL, &[]);
                m.control(out_setup(c.len(), 0), &c, v);
                m.control(in_setup(4), &[], v);
            }),
            ("does not use it", |m, v| {
                assert_eq!(quiet(m, 0x999, 0, &[], v), Reply::Stall);
            }),
            ("does not use it", |m, v| {
                assert_eq!(quiet(m, CLASS_LA, 5, &[0], v), Reply::Stall);
            }),
            ("read version with 1 bytes", |m, v| {
                ask(m, CLASS_CORE, VERB_VERSION, &[0], v);
            }),
            ("read serial number with 1 bytes", |m, v| {
                ask(m, CLASS_CORE, VERB_SERIAL, &[0], v);
            }),
            ("start with 1 bytes", |m, v| {
                quiet(m, CLASS_LA, VERB_START, &[0], v);
            }),
            ("stop with 1 bytes", |m, v| {
                quiet(m, CLASS_LA, VERB_STOP, &[0], v);
            }),
            ("configure with 1 bytes", |m, v| {
                let c = command(CLASS_LA, VERB_CONFIGURE, &[1]);
                assert_eq!(m.control(out_setup(c.len(), 0), &c, v), Reply::Stall);
            }),
            ("first pin with 0 bytes", |m, v| {
                assert_eq!(quiet(m, CLASS_LA, VERB_FIRST_PIN, &[], v), Reply::Stall);
            }),
            ("the board has 16", |m, v| {
                assert_eq!(quiet(m, CLASS_LA, VERB_FIRST_PIN, &[16], v), Reply::Stall);
            }),
            ("first pin 3", |m, v| {
                quiet(m, CLASS_LA, VERB_FIRST_PIN, &[3], v);
            }),
            ("alternate map with 0 bytes", |m, v| {
                assert_eq!(quiet(m, CLASS_LA, VERB_ALT, &[], v), Reply::Stall);
            }),
            ("alternate map 1", |m, v| {
                quiet(m, CLASS_LA, VERB_ALT, &[1], v);
            }),
            ("start before configure", |m, v| {
                quiet(m, CLASS_LA, VERB_START, &[], v);
            }),
            ("stop with no capture", |m, v| {
                quiet(m, CLASS_LA, VERB_STOP, &[], v);
            }),
            ("start during a capture", |m, v| {
                configure(m, 1_000_000, 8, v);
                quiet(m, CLASS_LA, VERB_START, &[], v);
                quiet(m, CLASS_LA, VERB_START, &[], v);
            }),
            ("configure during a capture", |m, v| {
                configure(m, 1_000_000, 8, v);
                quiet(m, CLASS_LA, VERB_START, &[], v);
                configure(m, 1_000_000, 8, v);
            }),
            ("with 0 channels", |m, v| {
                configure(m, 1_000_000, 0, v);
            }),
            ("with 17 channels", |m, v| {
                configure(m, 1_000_000, 17, v);
            }),
            ("OUT transfer", |m, v| {
                assert_eq!(m.out(2, &[0], v), Reply::Stall);
            }),
            ("the samples come from 1", |m, v| {
                assert_eq!(m.in_transfer(2, 512, v), Reply::Stall);
            }),
            ("before any start", |m, v| {
                m.in_submitted(1, 512);
                assert_eq!(m.in_transfer(1, 512, v), Reply::Nak);
            }),
            ("whole packets", |m, v| {
                m.in_transfer(1, 100, v);
            }),
        ];
        for (what, f) in cases {
            fires(&mut *board(&[]), f, what);
        }
    }

    #[test]
    fn a_failed_command_stays_underway_and_the_next_stalls() {
        let mut m = board(&[("stall", "serial")]);
        let mut v = Violations::default();
        let c = command(CLASS_CORE, VERB_SERIAL, &[]);
        assert_eq!(m.control(out_setup(c.len(), 0), &c, &mut v), Reply::Stall);
        assert!(v.is_empty());
        let next = command(CLASS_CORE, VERB_VERSION, &[]);
        assert_eq!(m.control(out_setup(next.len(), 0), &next, &mut v), Reply::Stall);
        assert_eq!(v.drain().len(), 1);
        // The STALL cleared it.
        assert_eq!(m.control(out_setup(next.len(), 0), &next, &mut v), Reply::ack());
    }

    #[test]
    fn faults_do_what_they_say() {
        let mut v = Violations::default();
        let mut m = board(&[("stall-read", "configure"), ("in-limit", "5")]);
        let mut args = 1_000_000u32.to_le_bytes().to_vec();
        args.push(8);
        assert_eq!(ask(&mut *m, CLASS_LA, VERB_CONFIGURE, &args, &mut v), Reply::Stall);

        let mut m = board(&[("configure-len", "4")]);
        assert_eq!(configure(&mut *m, 1_000_000, 8, &mut v).len(), 4);

        let mut m = board(&[("serial", "474631"), ("version", "")]);
        assert_eq!(ask(&mut *m, CLASS_CORE, VERB_SERIAL, &[], &mut v), Reply::Data(b"GF1".to_vec()));
        assert_eq!(ask(&mut *m, CLASS_CORE, VERB_VERSION, &[], &mut v), Reply::Data(Vec::new()));

        let mut m = board(&[("stall", "stop")]);
        configure(&mut *m, 1_000_000, 8, &mut v);
        quiet(&mut *m, CLASS_LA, VERB_START, &[], &mut v);
        assert_eq!(quiet(&mut *m, CLASS_LA, VERB_STOP, &[], &mut v), Reply::Stall);

        let mut m = board(&[("in-stall", "1"), ("in-limit", "3")]);
        configure(&mut *m, 1_000_000, 8, &mut v);
        quiet(&mut *m, CLASS_LA, VERB_START, &[], &mut v);
        assert!(matches!(m.in_transfer(1, 512, &mut v), Reply::Data(_)));
        assert_eq!(m.in_transfer(1, 512, &mut v), Reply::Stall);
        assert_eq!(m.in_transfer(1, 512, &mut v), Reply::Nak);

        let mut m = board(&[("stall_string", "3")]);
        let read = |i: u16| Setup {
            request_type: 0x80,
            request: 6,
            value: 0x0300 | i,
            index: 0x0409,
            length: 255,
        };
        assert_eq!(m.fail_standard(read(3)), Some(StandardFault::Stall));
        assert_eq!(m.fail_standard(read(1)), None);
        let mut m = board(&[("short_string", "3")]);
        assert_eq!(m.fail_standard(read(3)), Some(StandardFault::Short(2)));
        assert!(v.is_empty(), "{:?}", v.drain());
    }

    #[test]
    fn options_are_checked() {
        for bad in [
            &[("nope", "1")][..],
            &[("in-limit", "x")][..],
            &[("wide", "2")][..],
            &[("stall", "dance")][..],
            &[("stall-read", "dance")][..],
            &[("serial", "abc")][..],
            &[("version", "zz")][..],
            &[("stall_string", "9")][..],
        ] {
            assert!(build(&opts(bad)).is_err(), "{bad:?}");
        }
    }

    #[test]
    fn the_descriptors_are_those_of_the_firmware() {
        let d = firmware_descriptors();
        assert_eq!((d.vendor_id(), d.product_id(), d.bcd_device()), (0x1d50, 0x60e6, 0x0100));
        assert_eq!(d.device_class(), [0, 0, 0]);
        let c = d.config_by_value(1).unwrap();
        assert_eq!(c[2], 41, "wTotalLength");
        assert_eq!(c[8], 250, "bMaxPower");
        assert_eq!(Descriptors::interfaces(c).len(), 2);
        let eps = Descriptors::endpoints(c);
        assert_eq!((eps[0].address, eps[0].max_packet), (0x81, 512));
        assert_eq!((eps[1].address, eps[1].max_packet), (0x02, 512));
        assert_eq!(serial_string(), "000000000000000057cc67e630187657");
    }

    #[test]
    fn reset_starts_over() {
        let mut m = board(&[]);
        let mut v = Violations::default();
        configure(&mut *m, 1_000_000, 8, &mut v);
        quiet(&mut *m, CLASS_LA, VERB_START, &[], &mut v);
        m.reset();
        quiet(&mut *m, CLASS_LA, VERB_START, &[], &mut v);
        assert_eq!(v.drain().len(), 1, "start before configure, after the reset");
    }
}
