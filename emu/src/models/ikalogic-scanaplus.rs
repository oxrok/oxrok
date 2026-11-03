// Counterpart in sigrok:
//   libsigrok/src/hardware/ikalogic-scanaplus/api.c (the ids, the product string, the open sequence),
//   libsigrok/src/hardware/ikalogic-scanaplus/protocol.c (the EEPROM id, the FPGA writes, the compressed sample format),
//   libsigrok/src/hardware/ikalogic-scanaplus/protocol.h,
//   Copyright (C) 2013 Uwe Hermann <uwe@hermann-uwe.de>,
//   with commits by Lars-Peter Clausen, Bert Vermeulen, Daniel Elstner,
//   Gerhard Sittig, Martin Ling, Soeren Apel, Marc Schink
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The IKALOGIC `ScanaPLUS` as libsigrok's `ikalogic-scanaplus` driver
//! sees it: an FT232H (`0403:6014`) in synchronous FIFO mode in front
//! of an FPGA that samples nine channels at 100 MHz and sends them run
//! length compressed.
//!
//! Every fact is taken from the C driver; no datasheet, firmware or
//! recording of the device was available. What it does not pin down is
//! marked "assumed" and is the model's choice.
//!
//! **The chip.** `api.c` opens `0403:6014` with the product string
//! `SCANAPLUS` (`USB_VENDOR_ID`, `USB_DEVICE_ID`, `USB_IPRODUCT`) and
//! puts port A into `BITMODE_SYNCFF`, which only the FT2232H's port A
//! and the FT232H have; `0x6014` is the FT232H's own product id, so the
//! chip is taken to be an FT232H (bcdDevice 0x0900). The manufacturer
//! string `IKALOGIC` and the absent serial number are assumed: libftdi
//! is asked for neither. The EEPROM is the shell's FT232H image with
//! the device id in words 16 and 17 (`protocol.c`
//! `scanaplus_get_device_id()`: word 16's low byte, its high byte, word
//! 17's low byte), the checksum recomputed. `devid=` sets the three
//! bytes as stored; the driver clears bit 7 of each before sending them
//! ("we've seen both in actual `ScanaPLUS` devices"), so the default
//! `1ca357` has bit 7 set in its middle byte. Word 17's high byte is
//! assumed 0.
//!
//! **The FPGA's registers.** Every host write is a run of (register,
//! value) pairs, register with bit 7 set (`protocol.c`):
//! - `0x88` the channel configuration: `0x40` normal, `0x50`, `0x48`,
//!   `0x58` the differential pairs (the comment in
//!   `scanaplus_start_acquisition()`), and `0x41`, which
//!   `scanaplus_init()` writes first and clears again;
//! - `0x89`, `0x8a` the thresholds of channels 1-4 and 5-9: the five
//!   values of the table at the top of `protocol.c`, and `0x64`, which
//!   `scanaplus_init()` writes;
//! - `0x8d` a sequence `scanaplus_init()` writes: `01 05 01 02`, then 57
//!   times `06 02`;
//! - `0x8c`, `0x8e`, `0x8f` the three bytes of the device id, bit 7
//!   clear, or all three 0 (`scanaplus_clear_device_id()`).
//!
//! What the registers do is not known. Assumed, and the model's: bit 0
//! of `0x88` resets the FPGA's configuration (the `0x8d` sequence
//! starts over and the stream stops); the FPGA starts sending once it
//! is configured (the whole `0x8d` sequence) and has been sent the
//! EEPROM's id, cleared, and sent it again, which is the end of
//! `scanaplus_start_acquisition()`, the last thing the driver writes
//! before it reads. The driver never reads before that, so when the
//! real FPGA starts cannot show in a transaction.
//!
//! **The samples.** `scanaplus_uncompress_block()` takes the stream as
//! pairs of bytes: the first holds a count of samples in bits 1 to 7
//! and channel 9 in bit 0, the second channels 1 to 8. The model sends
//! pair `k` with the count `COUNTS[k % 7]` (1 to 127), channel 9 bit 8
//! of `k` and channels 1 to 8 the low byte of `k`, from the start of
//! the stream on. The FIFO is always full: every bulk IN transfer is
//! filled, one per submission, which libftdi's synchronous reads make
//! the device's natural pace. The driver drops its first read
//! whatever it holds (the FPGA's "dummy" samples after a start); the
//! model sends no dummy data, which that drop would hide anyway.
//! Out of sync FIFO mode, or before the stream starts, the chip has
//! nothing: two status bytes alone. The driver sends no stop: the
//! stream runs on after a capture, until a bus reset or `0x88` bit 0.
//! Emulation artifacts: the full FIFO at every submission (a real
//! FT232H's transfers may end short, on its latency timer, when the
//! FPGA's compressed stream is slower than USB) and the start point.
//!
//! What the driver does with it (`protocol.c`
//! `scanaplus_receive_data()`): libftdi reads in transfers of 16 KiB
//! (its Linux limit on `ftdi_read_data_set_chunksize()`'s 64 KiB), the
//! first `ftdi_read_data()` of 64 KiB is dropped whatever its length,
//! and each later one is decoded whole. With both limits set the sample
//! limit is tested first, so a read that passes both ends on the sample
//! limit even when the time limit is the smaller.
//!
//! **The checks** on top of the FTDI shell's: writes outside sync FIFO
//! mode, of an odd length, to a register not listed above or of a value
//! not listed for it; the `0x8d` sequence out of order or longer than
//! 57 pulses; an id byte with bit 7 set or other than the EEPROM's; an
//! acquisition started (id cleared and sent again) before the FPGA is
//! configured; a bit mode other than reset and sync FIFO.
//!
//! Options: `devid=XXXXXX` (the EEPROM's three id bytes, hex), `product=`
//! (the product string; anything but `SCANAPLUS` is a plain FT232H the
//! driver does not find), and the shell's faults ([`Faults`]):
//! `stall=REQUEST[/WVALUE]`, `in-stall=N`, `idle=N`.

use crate::model::{Descriptors, DeviceModel, Reply, Setup, Violations};
use crate::shells::ftdi::{
    Chip, Event, Faults, Fill, Ftdi, FtdiConfig, FtdiPorts, PortState, bitmode, eeprom_checksum,
};
use crate::{Error, Result};

/// The model name.
pub const NAME: &str = "ikalogic-scanaplus";
/// What `oxrok-emu list` says about it.
pub const ABOUT: &str = "IKALOGIC ScanaPLUS: FT232H in sync FIFO mode, an FPGA streaming \
     compressed 9 channel samples; devid=XXXXXX (EEPROM id bytes), product=; the FTDI shell's \
     faults: stall=REQUEST[/WVALUE], in-stall=N, idle=N";

/// The product string the driver opens (`api.c` `USB_IPRODUCT`).
pub const PRODUCT: &str = "SCANAPLUS";

/// The EEPROM's id bytes by default: bit 7 set in the middle one.
const DEFAULT_DEVID: [u8; 3] = [0x1c, 0xa3, 0x57];

/// The sample counts of the pairs, cycled.
/// Seven of them against the 512 pairs of the other bits, so that the
/// stream repeats only every 3584 pairs and a transfer sent twice or
/// out of order shows.
pub const COUNTS: [u8; 7] = [1, 2, 1, 3, 127, 4, 2];

/// The registers.
mod reg {
    pub const CONFIG: u8 = 0x88;
    pub const THRESHOLD_LOW: u8 = 0x89;
    pub const THRESHOLD_HIGH: u8 = 0x8a;
    pub const ID0: u8 = 0x8c;
    pub const SEQUENCE: u8 = 0x8d;
    pub const ID1: u8 = 0x8e;
    pub const ID2: u8 = 0x8f;
}

/// `0x88`'s values: normal, the three differential settings, and the
/// reset `scanaplus_init()` writes.
const CONFIG_VALUES: [u8; 5] = [0x40, 0x50, 0x48, 0x58, 0x41];
/// The thresholds: the table of `protocol.c` and `scanaplus_init()`'s 0x64.
const THRESHOLDS: [u8; 6] = [0x2e, 0x39, 0x45, 0x6c, 0x7f, 0x64];
/// The head of the `0x8d` sequence, then `PULSES` times `06 02`.
const SEQUENCE_HEAD: [u8; 4] = [0x01, 0x05, 0x01, 0x02];
const PULSES: usize = 57;
const SEQUENCE_LEN: usize = SEQUENCE_HEAD.len() + 2 * PULSES;

/// A `ScanaPLUS` from the `-o` options.
///
/// # Errors
///
/// On an unknown option or a bad value.
pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>> {
    let mut devid = DEFAULT_DEVID;
    let mut product = PRODUCT.to_owned();
    let mut faults = Faults::default();
    for (k, v) in options {
        match k.as_str() {
            "devid" => devid = parse_devid(v)?,
            "product" => product.clone_from(v),
            other => {
                if !faults.option(other, v)? {
                    return Err(Error(format!("{NAME} has no option {other}")));
                }
            }
        }
    }
    let mut model = ScanaPlus::new(devid, &product);
    model.chip.set_faults(faults);
    Ok(Box::new(model))
}

fn parse_devid(s: &str) -> Result<[u8; 3]> {
    let bad = || Error(format!("devid={s}: three bytes in hex, as 1ca357"));
    if s.len() != 6 {
        return Err(bad());
    }
    let n = u32::from_str_radix(s, 16).map_err(|_| bad())?;
    Ok([(n >> 16) as u8, (n >> 8) as u8, n as u8])
}

/// The board's FT232H configuration.
pub fn config(product: &str) -> FtdiConfig {
    let mut c = FtdiConfig::new(Chip::Ft232h);
    c.manufacturer = Some("IKALOGIC".to_owned());
    c.description = (!product.is_empty()).then(|| product.to_owned());
    c.serial = None;
    c
}

/// Byte `i` of the stream.
pub fn stream_byte(i: u64) -> u8 {
    let k = i / 2;
    if i.is_multiple_of(2) {
        (COUNTS[(k % 7) as usize] << 1) | ((k >> 8) & 1) as u8
    } else {
        k as u8
    }
}

/// The FPGA behind port A.
#[derive(Clone, Debug)]
pub struct Fpga {
    /// The id the driver must send: the EEPROM's bytes, bit 7 clear.
    id: [u8; 3],
    /// The id registers.
    id_regs: [u8; 3],
    /// Where the `0x8d` sequence is; `None` once it went wrong.
    sequence: Option<usize>,
    /// The id was sent since the last reset (then cleared: `cleared`).
    id_sent: bool,
    cleared: bool,
    /// Sending samples.
    streaming: bool,
    /// Stream bytes sent.
    sent: u64,
}

impl Fpga {
    fn new(eeprom_id: [u8; 3]) -> Fpga {
        Fpga {
            id: eeprom_id.map(|b| b & 0x7f),
            id_regs: [0; 3],
            sequence: Some(0),
            id_sent: false,
            cleared: false,
            streaming: false,
            sent: 0,
        }
    }

    /// Whether the samples are flowing.
    pub fn streaming(&self) -> bool {
        self.streaming
    }

    fn configured(&self) -> bool {
        self.sequence == Some(SEQUENCE_LEN)
    }

    fn register(&mut self, r: u8, value: u8, v: &mut Violations) {
        match r {
            reg::CONFIG => {
                if !CONFIG_VALUES.contains(&value) {
                    v.report(format!(
                        "register 0x88 = {value:#04x}: 0x40, 0x50, 0x48, 0x58 (protocol.c) or the \
                         init's 0x41"
                    ));
                }
                if value & 1 != 0 {
                    // Assumed: the FPGA's reset (see the module doc).
                    self.sequence = Some(0);
                    self.id_sent = false;
                    self.cleared = false;
                    self.streaming = false;
                }
            }
            reg::THRESHOLD_LOW | reg::THRESHOLD_HIGH => {
                if !THRESHOLDS.contains(&value) {
                    v.report(format!(
                        "threshold register {r:#04x} = {value:#04x}: one of 0x2e, 0x39, 0x45, \
                         0x6c, 0x7f (protocol.c) or the init's 0x64"
                    ));
                }
            }
            reg::SEQUENCE => self.sequence_step(value, v),
            reg::ID0 | reg::ID1 | reg::ID2 => self.id_byte(r, value, v),
            other => v.report(format!(
                "write to register {other:#04x}: the ScanaPLUS driver writes 0x88 to 0x8a and \
                 0x8c to 0x8f"
            )),
        }
    }

    fn sequence_step(&mut self, value: u8, v: &mut Violations) {
        let Some(at) = self.sequence else { return };
        let want = match at {
            _ if at < SEQUENCE_HEAD.len() => SEQUENCE_HEAD[at],
            _ if at < SEQUENCE_LEN => {
                if (at - SEQUENCE_HEAD.len()).is_multiple_of(2) {
                    0x06
                } else {
                    0x02
                }
            }
            _ => {
                v.report(format!(
                    "register 0x8d = {value:#04x} after the init's sequence of {PULSES} pulses"
                ));
                self.sequence = None;
                return;
            }
        };
        if value == want {
            self.sequence = Some(at + 1);
        } else {
            v.report(format!(
                "register 0x8d = {value:#04x} at step {at} of the init's sequence, which has \
                 {want:#04x} there"
            ));
            self.sequence = None;
        }
    }

    fn id_byte(&mut self, r: u8, value: u8, v: &mut Violations) {
        let i = match r {
            reg::ID0 => 0,
            reg::ID1 => 1,
            _ => 2,
        };
        if value & 0x80 != 0 {
            v.report(format!(
                "device id register {r:#04x} = {value:#04x}: the driver clears bit 7 \
                 (scanaplus_get_device_id)"
            ));
        } else if value != 0 && value != self.id[i] {
            v.report(format!(
                "device id register {r:#04x} = {value:#04x}: the EEPROM's id has {:#04x}",
                self.id[i]
            ));
        }
        self.id_regs[i] = value;
        if r != reg::ID2 {
            return;
        }
        // In this order, so that an id of three zero bytes works too.
        if self.cleared && self.id_regs == self.id {
            if self.configured() {
                self.streaming = true;
            } else {
                v.report(
                    "acquisition started (id cleared and sent again) before the init's 0x8d \
                     sequence",
                );
            }
            self.cleared = false;
        } else if self.id_sent && self.id_regs == [0; 3] {
            self.cleared = true;
        } else if self.id_regs == self.id {
            self.id_sent = true;
        }
    }
}

impl FtdiPorts for Fpga {
    fn changed(&mut self, _: usize, event: Event, state: &PortState, v: &mut Violations) {
        if event == Event::Bitmode && !matches!(state.bitmode, bitmode::RESET | bitmode::SYNCFF) {
            v.report(format!(
                "bit mode {:#04x}: the ScanaPLUS runs in sync FIFO mode (0x40)",
                state.bitmode
            ));
        }
    }

    fn write(&mut self, _: usize, data: &[u8], state: &PortState, v: &mut Violations) -> Reply {
        if state.bitmode != bitmode::SYNCFF {
            v.report(format!(
                "{} bytes written in bit mode {:#04x}: the FPGA sits on the sync FIFO",
                data.len(),
                state.bitmode
            ));
            return Reply::ack();
        }
        if !data.len().is_multiple_of(2) {
            v.report(format!(
                "write of {} bytes: the FPGA takes (register, value) pairs",
                data.len()
            ));
        }
        for &[r, value] in data.as_chunks::<2>().0 {
            if r & 0x80 == 0 {
                v.report(format!("write of {r:#04x} where a register (bit 7 set) belongs"));
                continue;
            }
            self.register(r, value, v);
        }
        Reply::ack()
    }

    fn read(&mut self, _: usize, max: usize, state: &PortState, _: &mut Violations) -> Fill {
        if state.bitmode != bitmode::SYNCFF || !self.streaming {
            return Fill::Idle;
        }
        let start = self.sent;
        self.sent += max as u64;
        Fill::Data((start..self.sent).map(stream_byte).collect())
    }

    fn pins(&mut self, _: usize, _: &PortState, _: &mut Violations) -> u8 {
        0
    }
}

/// The emulated `ScanaPLUS`.
pub struct ScanaPlus {
    chip: Ftdi,
    fpga: Fpga,
}

impl ScanaPlus {
    /// A `ScanaPLUS` whose EEPROM holds `devid`, with product string `product`.
    pub fn new(devid: [u8; 3], product: &str) -> ScanaPlus {
        let mut chip = Ftdi::new(config(product));
        let mut words = chip.eeprom().to_vec();
        words[16] = u16::from_le_bytes([devid[0], devid[1]]);
        words[17] = u16::from(devid[2]);
        let n = words.len();
        words[n - 1] = eeprom_checksum(&words[..n - 1]);
        chip.set_eeprom(words);
        ScanaPlus { chip, fpga: Fpga::new(devid) }
    }

    /// The shell.
    pub fn ftdi(&self) -> &Ftdi {
        &self.chip
    }

    /// The FPGA.
    pub fn fpga(&self) -> &Fpga {
        &self.fpga
    }
}

impl DeviceModel for ScanaPlus {
    fn name(&self) -> &'static str {
        NAME
    }

    fn descriptors(&self) -> &Descriptors {
        self.chip.descriptors()
    }

    fn control(&mut self, setup: Setup, data: &[u8], v: &mut Violations) -> Reply {
        self.chip.control(&mut self.fpga, setup, data, v)
    }

    fn out(&mut self, ep: u8, data: &[u8], v: &mut Violations) -> Reply {
        self.chip.out(&mut self.fpga, ep, data, v)
    }

    fn in_transfer(&mut self, ep: u8, max: usize, v: &mut Violations) -> Reply {
        self.chip.in_transfer(&mut self.fpga, ep, max, v)
    }

    fn reset(&mut self) {
        // A bus reset leaves the FPGA alone; a fresh model is a power on.
        self.chip.reset();
    }

    fn log_digest_above(&self) -> Option<usize> {
        Some(crate::txlog::HASH_ABOVE)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn out(request: u8, value: u16, index: u16) -> Setup {
        Setup { request_type: 0x40, request, value, index, length: 0 }
    }

    fn eeprom(index: u16) -> Setup {
        Setup { request_type: 0xc0, request: 0x90, value: 0, index, length: 2 }
    }

    /// The writes of `scanaplus_init()` and `scanaplus_start_acquisition()`.
    fn driver_writes(id: [u8; 3]) -> Vec<Vec<u8>> {
        let send = |w: &mut Vec<Vec<u8>>, id: [u8; 3]| {
            w.push(vec![0x8c, id[0]]);
            w.push(vec![0x8e, id[1]]);
            w.push(vec![0x8f, id[2]]);
        };
        let mut w = vec![
            vec![0x88, 0x41],
            vec![0x89, 0x64, 0x8a, 0x64],
            vec![0x88, 0x41],
            vec![0x88, 0x40],
            vec![0x8d, 0x01, 0x8d, 0x05, 0x8d, 0x01, 0x8d, 0x02],
        ];
        for _ in 0..57 {
            w.push(vec![0x8d, 0x06]);
            w.push(vec![0x8d, 0x02]);
        }
        send(&mut w, id);
        w.push(vec![0x88, 0x40]);
        w.push(vec![0x89, 0x7f, 0x8a, 0x7f]);
        w.push(vec![0x88, 0x40]);
        send(&mut w, [0; 3]);
        send(&mut w, id);
        w
    }

    /// Open as `dev_open()` does, through to the id read.
    fn open(dev: &mut ScanaPlus, v: &mut Violations) -> [u8; 3] {
        for s in [out(0, 0, 1), out(0, 1, 1), out(0, 2, 1), out(0x0b, 0x00ff, 1)] {
            assert_eq!(dev.control(s, &[], v), Reply::ack());
        }
        assert_eq!(dev.control(out(0x0b, 0x40ff, 1), &[], v), Reply::ack());
        assert_eq!(dev.control(out(0x09, 2, 1), &[], v), Reply::ack());
        let word = |dev: &mut ScanaPlus, v: &mut Violations, i| {
            let Reply::Data(d) = dev.control(eeprom(i), &[], v) else { panic!() };
            u16::from_le_bytes([d[0], d[1]])
        };
        let (a, b) = (word(dev, v, 16), word(dev, v, 17));
        [(a & 0x7f) as u8, ((a >> 8) & 0x7f) as u8, (b & 0x7f) as u8]
    }

    #[test]
    fn streams_after_the_drivers_start() {
        let mut dev = ScanaPlus::new(DEFAULT_DEVID, PRODUCT);
        let mut v = Violations::default();
        let id = open(&mut dev, &mut v);
        assert_eq!(id, [0x1c, 0x23, 0x57]);
        assert_eq!(dev.in_transfer(1, 512, &mut v), Reply::Data(vec![0x01, 0x60]));
        let writes = driver_writes(id);
        let (last, init) = writes.split_last().unwrap();
        for w in init {
            assert_eq!(dev.out(2, w, &mut v), Reply::ack());
        }
        assert!(!dev.fpga().streaming());
        dev.out(2, last, &mut v);
        assert!(dev.fpga().streaming());
        let Reply::Data(d) = dev.in_transfer(1, 65536, &mut v) else { panic!() };
        assert_eq!(d.len(), 65536);
        // Pair 0: one sample, channel 9 low, channels 1-8 0; pair 4: 127.
        assert_eq!(&d[..12], &[0x01, 0x60, 0x02, 0x00, 0x04, 0x01, 0x02, 0x02, 0x06, 0x03, 0xfe, 0x04]);
        // Pair 256 has channel 9 high; packets are 510 bytes of data.
        assert_eq!(stream_byte(512), (COUNTS[256 % 7] << 1) | 1);
        assert_eq!(&d[512..515], &[0x01, 0x60, stream_byte(510)]);
        assert!(v.is_empty(), "{v:?}");
        // A new init stops the stream until the next start.
        dev.out(2, &[0x88, 0x41], &mut v);
        assert_eq!(dev.in_transfer(1, 512, &mut v), Reply::Data(vec![0x01, 0x60]));
        assert!(v.is_empty(), "{v:?}");
    }

    #[test]
    fn eeprom_holds_the_id() {
        let dev = ScanaPlus::new([0x81, 0x02, 0x83], PRODUCT);
        let w = dev.ftdi().eeprom();
        assert_eq!((w[16], w[17]), (0x0281, 0x0083));
        assert_eq!(eeprom_checksum(&w[..w.len() - 1]), w[w.len() - 1]);
        assert_eq!(dev.fpga().id, [0x01, 0x02, 0x03]);
        let d = dev.descriptors();
        assert_eq!(d.strings, ["IKALOGIC", "SCANAPLUS"]);
        assert_eq!(&d.device[8..14], &[0x03, 0x04, 0x14, 0x60, 0x00, 0x09]);
    }

    #[test]
    fn flags_what_the_driver_never_sends() {
        let mut dev = ScanaPlus::new(DEFAULT_DEVID, PRODUCT);
        let mut v = Violations::default();
        dev.out(2, &[0x88, 0x40], &mut v);
        assert_eq!(v.drain().len(), 1, "a write before sync FIFO mode");
        let id = open(&mut dev, &mut v);
        assert!(v.is_empty(), "{v:?}");
        let mut one = |data: &[u8], what: &str| {
            dev.out(2, data, &mut v);
            assert_eq!(v.drain().len(), 1, "{what}");
        };
        one(&[0x88, 0x40, 0x89], "an odd length");
        one(&[0x08, 0x40], "no register bit");
        one(&[0x8b, 0x00], "an unknown register");
        one(&[0x88, 0x42], "a config value");
        one(&[0x89, 0x40], "a threshold");
        one(&[0x8c, 0x9c], "an id byte with bit 7");
        one(&[0x8c, 0x1d], "a wrong id byte");
        one(&[0x8d, 0x05], "the sequence out of order");
        // A start before the sequence.
        dev.out(2, &[0x88, 0x41], &mut v);
        for w in [[0x8c, id[0]], [0x8e, id[1]], [0x8f, id[2]], [0x8c, 0], [0x8e, 0], [0x8f, 0]] {
            dev.out(2, &w, &mut v);
        }
        assert!(v.is_empty(), "{v:?}");
        dev.out(2, &[0x8c, id[0], 0x8e, id[1], 0x8f, id[2]], &mut v);
        assert_eq!(v.drain().len(), 1, "a start before the sequence");
        assert!(!dev.fpga().streaming());
        // Too many pulses.
        dev.out(2, &[0x88, 0x41], &mut v);
        for w in &driver_writes(id)[4..5 + 2 * 57] {
            dev.out(2, w, &mut v);
        }
        assert!(v.is_empty(), "{v:?}");
        dev.out(2, &[0x8d, 0x06], &mut v);
        assert_eq!(v.drain().len(), 1, "a 58th pulse");
        // MPSSE.
        dev.control(out(0x0b, 0x02ff, 1), &[], &mut v);
        assert_eq!(v.drain().len(), 1, "MPSSE");
    }

    #[test]
    fn options() {
        let o = |k: &str, v: &str| (k.to_owned(), v.to_owned());
        assert!(build(&[o("devid", "0102")]).is_err());
        assert!(build(&[o("devid", "01020x")]).is_err());
        assert!(build(&[o("nosuch", "1")]).is_err());
        let m = build(&[o("devid", "ffffff"), o("product", "Single RS232-HS"), o("idle", "2")])
            .unwrap();
        assert_eq!(m.descriptors().strings[1], "Single RS232-HS");
        assert_eq!(parse_devid("1ca357").unwrap(), DEFAULT_DEVID);
        // An id of zeros starts the stream too.
        let mut dev = ScanaPlus::new([0x80, 0, 0], PRODUCT);
        let mut v = Violations::default();
        let id = open(&mut dev, &mut v);
        for w in driver_writes(id) {
            dev.out(2, &w, &mut v);
        }
        assert!(dev.fpga().streaming());
        assert!(v.is_empty(), "{v:?}");
    }
}
