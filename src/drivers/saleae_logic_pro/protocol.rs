// Counterpart in sigrok:
//   libsigrok/src/hardware/saleae-logic-pro/api.c (the firmware upload),
//   libsigrok/src/hardware/saleae-logic-pro/protocol.c,
//   libsigrok/src/hardware/saleae-logic-pro/protocol.h,
//   Copyright (C) 2017 Jan Luebbe <jluebbe@lasnet.de>,
//   with commits by Uwe Hermann, Gerhard Sittig, Romain Tartière
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The wire protocol of the Saleae Logic Pro 16.
//!
//! **Firmware.** The board comes up as a Cypress FX3 boot loader with the
//! driver's own ids. `saleae-logicpro16-fx3.fw` is a list of parts, each
//! a 7 byte header (size LE16, address LE32, final flag) and its data, of
//! at most 4 KiB. Each part goes out as vendor request `0xa0` with the
//! address in `wValue` (low half) and `wIndex` (high half); the last,
//! empty and flagged final, starts the firmware, which renumerates as
//! "Saleae" / "Logic Pro".
//!
//! **Commands.** A command is one bulk OUT on endpoint 1 of 2 to 1024
//! bytes (a flag byte, the command, its arguments), and every command but
//! the reseed is answered by one bulk IN on endpoint 1. Both are XORed
//! with a 32 bit LFSR, byte `i` with bits `8 * (i % 4)` up, except bits
//! `0x28` of a command's first byte, and each direction steps the LFSR
//! once ([`Link::transact`]). The reseed zeroes the state, and 0 is a
//! fixed point, so the open runs in plain bytes. The step happens before
//! a command is sent, whether or not it arrives, and after an answer is
//! read, only if it was.
//!
//! **Authentication.** Each capture wakes a crypto chip on the FX3's I2C
//! bus, then asks it for a random number, two nonces and a signature with
//! fixed packets ([`Link::prepare`]). The LFSR is then keyed from the last
//! nonce and the signature's CRC. A failure is logged and the capture
//! goes on with the old state, as in `saleae_logic_pro_prepare()` (whether
//! a real board captures so is not known).
//!
//! **FPGA.** Register `0x7f` reads `0xaa` once the FPGA is configured;
//! otherwise the bitstream goes up in chunks of 1020 bytes
//! ([`Link::init`]).
//!
//! **Samples.** Bulk IN endpoint 2 carries 32 bit LE words, each 32
//! samples of one enabled channel, first sample in bit 31, the channels in
//! turn ([`Converter`]).

use crate::core::firmware;
use crate::core::{Error, Result};
use crate::usb::{ControlRequest, Transport, libusb_error_name};
use std::time::Duration;

const COMMAND_START_CAPTURE: u8 = 0x01;
const COMMAND_STOP_CAPTURE: u8 = 0x02;
const COMMAND_READ_EEPROM: u8 = 0x07;
const COMMAND_INIT_BITSTREAM: u8 = 0x7e;
const COMMAND_SEND_BITSTREAM: u8 = 0x7f;
const COMMAND_WRITE_REG: u8 = 0x80;
const COMMAND_READ_REG: u8 = 0x81;
const COMMAND_READ_TEMP: u8 = 0x86;
const COMMAND_WRITE_I2C: u8 = 0x87;
const COMMAND_READ_I2C: u8 = 0x88;
const COMMAND_WAKE_I2C: u8 = 0x89;
const COMMAND_READ_FW_VER: u8 = 0x8b;

const REG_ADC_IDX: u8 = 0x03;
const REG_ADC_VAL_LSB: u8 = 0x04;
const REG_ADC_VAL_MSB: u8 = 0x05;
const REG_STATUS: u8 = 0x40;
const REG_SCRATCH: u8 = 0x7f;
const SCRATCH_MAGIC: u8 = 0xaa;

const EP_COMMAND_OUT: u8 = 0x01;
const EP_COMMAND_IN: u8 = 0x81;
const COMMAND_TIMEOUT: Duration = Duration::from_millis(1000);
/// How long the chip takes to answer (`g_usleep()` in `crypto_*()`, "TODO:
/// Poll instead").
const CRYPTO_WAIT: Duration = Duration::from_millis(100);

const FW_HEADER_SIZE: usize = 7;
const FW_MAX_PART_SIZE: usize = 4 * 1024;
const FW_MAX_SIZE: usize = 256 * 1024;
const FW_TIMEOUT: Duration = Duration::from_millis(100);
const BITSTREAM_MAX_SIZE: usize = 512 * 1024;
const BITSTREAM_PART_SIZE: usize = 1020;
const BITSTREAM: &str = "saleae-logicpro16-fpga.bitstream";
const HINT: &str = "see docs/DEVICES.md";

/// The ADC writes of the open: address and value, in order, up to
/// register `0x15` (`saleae_logic_pro_init()`).
const ADC_INIT: [(u8, u16); 10] = [
    (0x11, 0x0444),
    (0x12, 0x0777),
    (0x25, 0x0000),
    (0x45, 0x0000),
    (0x2a, 0x1111),
    (0x2b, 0x1111),
    (0x46, 0x0004),
    (0x50, 0x0000),
    (0x55, 0x0020),
    (0x56, 0x0000),
];

/// The fixed chip packets, with the CRCs the driver hard codes.
const CRYPTO_RANDOM: [u8; 8] = [0x03, 0x07, 0x1b, 0x00, 0x00, 0x00, 0x24, 0xcd];
const CRYPTO_SIGN: [u8; 8] = [0x03, 0x07, 0x41, 0x80, 0x00, 0x00, 0x28, 0x05];
const CRYPTO_NONCE: [u8; 28] = {
    let mut p = [0; 28];
    p[0] = 0x03;
    p[1] = 0x1b;
    p[2] = 0x16;
    p[26] = 0x7d;
    p[27] = 0xe0;
    p
};

/// A file, as `sr_resource_load()` takes it: its two messages when it is
/// not there, a refusal when it is over `max` bytes.
fn load_resource(name: &str, max: usize) -> Result<Vec<u8>> {
    let image = firmware::read(name, HINT).inspect_err(|e| {
        if matches!(e, Error::Firmware(_)) {
            log::debug!("Failed to locate '{name}'.");
        }
        log::error!("Failed to open resource '{name}' (use loglevel 5/spew for details).");
    })?;
    if image.len() > max {
        log::error!("Size {} of '{name}' exceeds limit {max}.", image.len());
        return Err(Error::device(format!("{name} is too large")));
    }
    Ok(image)
}

/// `upload_firmware()`: the parts of `name` to an open boot loader.
pub fn upload_firmware(t: &mut dyn Transport, name: &str) -> Result<()> {
    let image = load_resource(name, FW_MAX_SIZE)?;
    log::info!("Uploading firmware '{name}'.");
    send_firmware(t, &image)?;
    log::info!("Firmware upload done.");
    Ok(())
}

/// The loop of `upload_firmware()`. A header that promises more than the
/// file holds stops it where it is; parts sent stay sent. An empty last
/// part that is not flagged final fails here too, where the C driver logs
/// "Missing final part." and returns the transfer's 0, `SR_OK`
/// (`api.c:111-116`).
fn send_firmware(t: &mut dyn Transport, image: &[u8]) -> Result<()> {
    let mut offset = 0;
    let mut last = None;
    while offset + FW_HEADER_SIZE <= image.len() {
        let h = &image[offset..offset + FW_HEADER_SIZE];
        let size = usize::from(u16::from_le_bytes([h[0], h[1]]));
        let address = u32::from_le_bytes([h[2], h[3], h[4], h[5]]);
        let is_final = h[6] != 0;
        if size > FW_MAX_PART_SIZE {
            log::error!("Part too large ({size}).");
            return Err(Error::device("firmware part too large"));
        }
        offset += FW_HEADER_SIZE;
        let Some(data) = image.get(offset..offset + size) else {
            log::error!("Truncated firmware file.");
            return Err(Error::device("truncated firmware file"));
        };
        #[allow(clippy::cast_possible_truncation)]
        let (value, index) = (address as u16, (address >> 16) as u16);
        t.control_out(ControlRequest::vendor(0xa0, value, index), data, FW_TIMEOUT).inspect_err(
            |e| log::error!("Unable to send firmware to device: {}.", libusb_error_name(e)),
        )?;
        if size == 0 {
            log::info!("Started firmware at 0x{address:x}.");
        } else {
            log::trace!("Uploaded {size} bytes.");
        }
        offset += size;
        last = Some((size, is_final));
    }
    if last != Some((0, true)) {
        log::error!("Missing final part.");
        return Err(Error::device("missing final firmware part"));
    }
    Ok(())
}

/// `iterate_lfsr()`.
#[must_use]
pub fn iterate_lfsr(lfsr: u32) -> u32 {
    let mut l = lfsr;
    for _ in 0..=(lfsr & 0x1f) + 34 {
        l = (l >> 1) | ((l ^ (l >> 1) ^ (l >> 21) ^ (l >> 31)) << 31);
    }
    l
}

/// The mask of byte `i`.
#[allow(clippy::cast_possible_truncation)]
fn mask(lfsr: u32, i: usize) -> u8 {
    (lfsr >> (i % 4 * 8)) as u8
}

/// `encrypt()`, without the LFSR's step.
#[must_use]
pub fn encrypt(lfsr: u32, data: &[u8]) -> Vec<u8> {
    data.iter()
        .enumerate()
        .map(|(i, &v)| {
            let m = mask(lfsr, i);
            if i == 0 { (v & 0x28) | ((v ^ m) & !0x28) } else { v ^ m }
        })
        .collect()
}

/// `decrypt()`, without the LFSR's step.
#[must_use]
pub fn decrypt(lfsr: u32, data: &[u8]) -> Vec<u8> {
    data.iter().enumerate().map(|(i, &v)| v ^ mask(lfsr, i)).collect()
}

/// The key `authenticate()` takes: the first 28 bytes of the nonce as LE
/// words, XOR the signature's CRC.
#[must_use]
pub fn auth_key(nonce: &[u8; 32], crc: [u8; 2]) -> u32 {
    let mut k = 0;
    for (i, &b) in nonce[..28].iter().enumerate() {
        k ^= u32::from(b) << (8 * (i % 4));
    }
    k ^ u32::from(u16::from_le_bytes(crc))
}

/// A sample rate's divider and pre-divider values (registers `0x0b` and
/// `0x14`), or `None` for a rate the driver does not know.
#[must_use]
pub fn rate_registers(samplerate: u64) -> Option<(u8, u8)> {
    Some(match samplerate {
        1_000_000 => (0x64, 0xff),
        2_000_000 => (0x32, 0xff),
        2_500_000 => (0x28, 0xff),
        10_000_000 => (0x0a, 0xff),
        25_000_000 => (0x04, 0x80),
        50_000_000 => (0x02, 0x40),
        _ => return None,
    })
}

/// A failure the driver reports at debug level, and returns.
fn failure(msg: String) -> Error {
    log::debug!("{msg}");
    Error::Device(msg)
}

/// A failure the driver reports at error level, and returns.
fn fatal(msg: String) -> Error {
    log::error!("{msg}");
    Error::Device(msg)
}

/// The transport, with the LFSR's state.
pub struct Link {
    transport: Box<dyn Transport>,
    lfsr: u32,
}

impl Link {
    /// A link on an open device with its interface claimed.
    pub fn new(transport: Box<dyn Transport>) -> Self {
        Link { transport, lfsr: 0 }
    }

    /// The transport, for the sample stream.
    pub fn transport(&mut self) -> &mut dyn Transport {
        self.transport.as_mut()
    }

    fn step(&mut self) {
        let next = iterate_lfsr(self.lfsr);
        log::trace!("Iterate 0x{:08x} -> 0x{next:08x}", self.lfsr);
        self.lfsr = next;
    }

    /// `transact()`. An `rsp_len` of 0 reads and drops the one byte every
    /// command but the reseed is answered with. nusb reads whole packets,
    /// so an answer over `rsp_len` is what libusb calls an overflow.
    fn transact(&mut self, req: &[u8], rsp_len: usize) -> Result<Vec<u8>> {
        let command = req[1];
        let encrypted = encrypt(self.lfsr, req);
        self.step();
        let sent = self
            .transport
            .bulk_out(EP_COMMAND_OUT, &encrypted, COMMAND_TIMEOUT)
            .inspect_err(|e| {
                log::debug!("Failed to send request 0x{command:02x}: {}.", libusb_error_name(e));
            })?;
        if sent != req.len() {
            return Err(failure(format!(
                "Failed to send request 0x{command:02x}: incorrect length {sent} != {}.",
                req.len()
            )));
        }
        if req[0] == 0x20 {
            return Ok(Vec::new());
        }
        let want = rsp_len.max(1);
        let data =
            self.transport.bulk_in(EP_COMMAND_IN, want, COMMAND_TIMEOUT).inspect_err(|e| {
                log::debug!(
                    "Failed to receive response to request 0x{command:02x}: {}.",
                    libusb_error_name(e)
                );
            })?;
        if data.len() > want {
            return Err(failure(format!(
                "Failed to receive response to request 0x{command:02x}: LIBUSB_ERROR_OVERFLOW."
            )));
        }
        if data.len() != want {
            return Err(failure(format!(
                "Failed to receive response to request 0x{command:02x}: incorrect length {} != \
                 {want}.",
                data.len()
            )));
        }
        let plain = decrypt(self.lfsr, &data);
        self.step();
        Ok(plain)
    }

    /// `reseed()`.
    fn reseed(&mut self) -> Result<()> {
        self.lfsr = 0;
        self.transact(&[0x20, 0x24, 0x4b, 0x35, 0x8e], 0).map(|_| ())
    }

    #[allow(clippy::cast_possible_truncation)]
    fn write_regs(&mut self, regs: &[(u8, u8)]) -> Result<()> {
        let mut req = vec![0x00, COMMAND_WRITE_REG, regs.len() as u8];
        for &(address, value) in regs {
            req.extend_from_slice(&[address, value]);
        }
        self.transact(&req, 0).map(|_| ())
    }

    fn write_reg(&mut self, address: u8, value: u8) -> Result<()> {
        self.write_regs(&[(address, value)])
    }

    fn read_reg(&mut self, address: u8) -> Result<u8> {
        Ok(self.transact(&[0x00, COMMAND_READ_REG, 1, address], 1)?[0])
    }

    fn write_adc(&mut self, address: u8, value: u16) -> Result<()> {
        let [lsb, msb] = value.to_le_bytes();
        self.write_regs(&[(REG_ADC_IDX, address), (REG_ADC_VAL_LSB, lsb), (REG_ADC_VAL_MSB, msb)])
    }

    fn read_eeprom(&mut self, address: u16, len: u16) -> Result<Vec<u8>> {
        let [al, ah] = address.to_le_bytes();
        let [ll, lh] = len.to_le_bytes();
        self.transact(&[0x00, COMMAND_READ_EEPROM, 0x33, 0x81, al, ah, ll, lh], usize::from(len))
    }

    /// `read_i2c()`: the `len` bytes after the status byte.
    fn read_i2c(&mut self, len: u8) -> Result<Vec<u8>> {
        let mut rsp =
            self.transact(&[0x00, COMMAND_READ_I2C, 0xc0, len, 0], 1 + usize::from(len))?;
        if rsp[0] != 0x02 {
            return Err(failure(format!("Failed to do I2C read (0x{:02x}).", rsp[0])));
        }
        rsp.remove(0);
        Ok(rsp)
    }

    #[allow(clippy::cast_possible_truncation)]
    fn write_i2c(&mut self, data: &[u8]) -> Result<()> {
        let mut req = vec![0x00, COMMAND_WRITE_I2C, 0xc0, data.len() as u8, 0];
        req.extend_from_slice(data);
        let rsp = self.transact(&req, 1)?;
        if rsp[0] != 0x02 {
            return Err(failure(format!("Failed to do I2C write (0x{:02x}).", rsp[0])));
        }
        Ok(())
    }

    /// `wake_i2c()`. The message names the byte the check reads; the C
    /// driver prints the one before it (`protocol.c:340-341`).
    fn wake_i2c(&mut self) -> Result<()> {
        let rsp = self.transact(&[0x00, COMMAND_WAKE_I2C], 1)?;
        if rsp[0] != 0x00 {
            return Err(failure(format!("Failed to do I2C wake trigger (0x{:02x}).", rsp[0])));
        }
        let rsp = self.read_i2c(4)?;
        if rsp[1] != 0x11 {
            return Err(failure(format!("Failed to do I2C wake read (0x{:02x}).", rsp[1])));
        }
        Ok(())
    }

    /// A chip command and its answer: a count byte, `len` bytes, the CRC.
    fn crypto(&mut self, packet: &[u8], len: u8) -> Result<Vec<u8>> {
        self.write_i2c(packet)?;
        std::thread::sleep(CRYPTO_WAIT);
        self.read_i2c(1 + len + 2)
    }

    /// `authenticate()`: keys the LFSR from the crypto chip.
    fn authenticate(&mut self) -> Result<()> {
        let first4 =
            |b: &[u8]| format!("0x{:02x} 0x{:02x} 0x{:02x} 0x{:02x}", b[0], b[1], b[2], b[3]);
        self.wake_i2c()?;
        let random = self.crypto(&CRYPTO_RANDOM, 32)?;
        log::debug!("random: {}", first4(&random[1..]));
        let mut nonce = [0; 32];
        for _ in 0..2 {
            let rsp = self.crypto(&CRYPTO_NONCE, 32)?;
            nonce.copy_from_slice(&rsp[1..33]);
            log::debug!("nonce: {}", first4(&nonce));
        }
        let rsp = self.crypto(&CRYPTO_SIGN, 64)?;
        log::debug!("sig: {}", first4(&rsp[1..]));
        let crc = [rsp[65], rsp[66]];
        log::debug!("sig crc: 0x{:02x} 0x{:02x}", crc[0], crc[1]);
        let key = auth_key(&nonce, crc);
        log::debug!("Authenticate 0x{:08x} -> 0x{key:08x}", self.lfsr);
        self.lfsr = key;
        Ok(())
    }

    /// `upload_bitstream()`.
    fn upload_bitstream(&mut self) -> Result<()> {
        let image = load_resource(BITSTREAM, BITSTREAM_MAX_SIZE)?;
        log::info!("Uploading bitstream '{BITSTREAM}'.");
        let rsp = self.transact(&[0x00, COMMAND_INIT_BITSTREAM], 1)?;
        if rsp[0] != 0x00 {
            return Err(fatal(format!("Failed to start bitstream upload (0x{:02x}).", rsp[0])));
        }
        self.send_bitstream(&image)
    }

    /// The chunks of the upload and the check of the scratch register. The
    /// message of a failed read back names the register's value; the C
    /// driver prints the init command's answer (`protocol.c:539-540`).
    #[allow(clippy::cast_possible_truncation)]
    fn send_bitstream(&mut self, image: &[u8]) -> Result<()> {
        for part in image.chunks(BITSTREAM_PART_SIZE) {
            log::trace!("Uploading {} bytes.", part.len());
            let [ll, lh] = (part.len() as u16).to_le_bytes();
            let mut req = vec![0x00, COMMAND_SEND_BITSTREAM, ll, lh];
            req.extend_from_slice(part);
            let rsp = self.transact(&req, 1)?;
            if rsp[0] != 0x00 {
                return Err(failure(format!("Failed to do bitstream upload (0x{:02x}).", rsp[0])));
            }
        }
        log::info!("Bitstream upload done.");
        self.write_reg(REG_SCRATCH, SCRATCH_MAGIC)?;
        let value = self.read_reg(REG_SCRATCH)?;
        if value != SCRATCH_MAGIC {
            return Err(fatal(format!("Failed FPGA register read-back (0x{value:02x} != 0xaa).")));
        }
        Ok(())
    }

    /// `saleae_logic_pro_init()`.
    pub fn init(&mut self) -> Result<()> {
        self.reseed()?;
        let version = self.transact(&[0x00, COMMAND_READ_FW_VER], 128)?;
        let text = version[..63].split(|&b| b == 0).next().unwrap_or_default();
        log::debug!("fw-version: {}", String::from_utf8_lossy(text));
        log::debug!("read serial");
        self.read_eeprom(0x08, 8)?;
        if self.read_reg(REG_SCRATCH)? == SCRATCH_MAGIC {
            log::info!("Skipping bitstream upload.");
        } else {
            self.upload_bitstream()?;
        }
        log::debug!("reset ADC");
        self.write_reg(0x00, 0x00)?;
        self.write_reg(0x00, 0x80)?;
        log::debug!("init ADC");
        for (address, value) in ADC_INIT {
            self.write_adc(address, value)?;
        }
        self.write_reg(0x15, 0x00)?;
        self.write_adc(0x0f, 0x0100)?;
        log::debug!("resets");
        for bit in [0x02, 0x04, 0x08] {
            self.write_reg(0x00, bit)?;
            self.write_reg(0x00, 0x00)?;
        }
        log::debug!("read dummy");
        for i in 0..8 {
            self.read_reg(0x41 + i)?;
        }
        log::debug!("read/write magic");
        let magic = self.read_eeprom(0x10, 16)?;
        for byte in magic {
            self.write_reg(0x17, byte)?;
        }
        let temperature = self.transact(&[0x00, COMMAND_READ_TEMP], 1)?[0].cast_signed();
        log::debug!("temperature = {temperature}");
        Ok(())
    }

    /// `saleae_logic_pro_prepare()` after its check of the rate: every
    /// failing request is ignored there. Returns the outcome of the last
    /// one, the start command.
    pub fn prepare(&mut self, channels: u16, divider: u8, pre_divider: u8) -> Result<()> {
        if let Err(e) = self.authenticate() {
            log::debug!("No authentication, capturing all the same: {e}.");
        }
        let [mask_lsb, mask_msb] = channels.to_le_bytes();
        #[allow(clippy::cast_possible_truncation)]
        let count = channels.count_ones() as u8;
        let _ = self.write_reg(0x15, 0x03);
        let _ = self.write_regs(&[(0x03, 0x0f), (0x04, 0x00), (0x05, 0x00)]);
        let _ = self.write_regs(&[
            (0x00, 0x00),
            (0x08, 0x00),
            (0x09, 0x00),
            (0x06, mask_lsb),
            (0x07, mask_msb),
            (0x0a, 0x00),
            (0x0b, divider),
            (0x0c, 0x00),
            (0x0d, 0x00),
            (0x0e, count),
            (0x12, 0x04),
            (0x13, 0x00),
            (0x14, pre_divider),
        ]);
        self.transact(&[0x00, COMMAND_START_CAPTURE], 2).map(|_| ())
    }

    /// `saleae_logic_pro_start()`.
    pub fn start(&mut self) {
        let _ = self.write_reg(0x00, 0x01);
    }

    /// `saleae_logic_pro_stop()`.
    pub fn stop(&mut self) {
        let _ = self.write_reg(0x00, 0x00);
        let _ = self.transact(&[0x00, COMMAND_STOP_CAPTURE], 2);
        if let Ok(status) = self.read_reg(REG_STATUS)
            && status != 0x20
        {
            log::error!("Capture error (status reg = 0x{status:02x}).");
        }
    }
}

/// `saleae_logic_pro_convert_data()`: words of the channels in turn to 16
/// bit samples. It takes what a transfer holds, where the C driver
/// converts all 16 KiB of its buffer (`protocol.c:906`). Bytes short of a
/// word and channels short of a batch wait for the next transfer.
pub struct Converter {
    masks: Vec<u16>,
    index: usize,
    batch: [u16; 32],
    partial: Vec<u8>,
}

impl Converter {
    /// A converter for the channels of `channels`.
    #[must_use]
    pub fn new(channels: u16) -> Self {
        let masks = (0..16).map(|i| 1 << i).filter(|m| channels & m != 0).collect();
        Converter { masks, index: 0, batch: [0; 32], partial: Vec::new() }
    }

    /// The samples, 2 bytes LE each, of the batches `data` completes.
    pub fn convert(&mut self, data: &[u8]) -> Vec<u8> {
        let mut out = Vec::with_capacity(data.len() / 4 * 64);
        if self.masks.is_empty() {
            return out;
        }
        self.partial.extend_from_slice(data);
        let words = self.partial.len() / 4;
        for word in self.partial[..words * 4].chunks_exact(4) {
            let samples = u32::from_le_bytes([word[0], word[1], word[2], word[3]]);
            let channel = self.masks[self.index];
            for (i, s) in self.batch.iter_mut().enumerate() {
                if samples >> (31 - i) & 1 != 0 {
                    *s |= channel;
                }
            }
            self.index += 1;
            if self.index == self.masks.len() {
                self.index = 0;
                for s in &self.batch {
                    out.extend_from_slice(&s.to_le_bytes());
                }
                self.batch = [0; 32];
            }
        }
        self.partial.drain(..words * 4);
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::usb::record::Replay;

    /// A link with `script` as its transactions.
    fn link(script: &str) -> (Link, Replay) {
        let text = format!(
            "# oxrok usb fixture 1\ndevice vid=21a9 pid=1006 version=0000 interfaces=0:ff:ff:ff\n\
             {script}"
        );
        let r = Replay::from_text(&text).unwrap();
        (Link::new(Box::new(r.clone())), r)
    }

    #[test]
    fn lfsr_and_scrambling_match_the_wire() {
        // 0 is the fixed point the open runs on.
        assert_eq!(iterate_lfsr(0), 0);
        assert_eq!(encrypt(0, &[0x00, 0x80, 1]), [0x00, 0x80, 1]);
        // The write of register 0x00 and its answer under the key of
        // tests/emu/saleae-logic-pro/debug-lines.out are undone by the
        // same state.
        let k = 0x727c_a563;
        let e = encrypt(k, &[0x00, 0x80, 1, 0x15, 0x03]);
        assert_eq!(decrypt(k, &e)[1..], [0x80, 1, 0x15, 0x03]);
        // The flag bits of byte 0 go plain whatever the state.
        assert_eq!(encrypt(0xffff_ffff, &[0x20])[0], 0xf7);
        assert_eq!(encrypt(0xffff_ffff, &[0x00])[0], 0xd7);
        // (lfsr & 0x1f) + 35 shifts, the count from the state before.
        assert_ne!(iterate_lfsr(k), k);
        assert_eq!(iterate_lfsr(1), iterate_lfsr_by_hand(1, 36));
        assert_eq!(iterate_lfsr(0x1f), iterate_lfsr_by_hand(0x1f, 66));
    }

    fn iterate_lfsr_by_hand(mut l: u32, shifts: u32) -> u32 {
        for _ in 0..shifts {
            let bit = (l & 1) ^ ((l >> 1) & 1) ^ ((l >> 21) & 1) ^ (l >> 31);
            l = (l >> 1) | (bit << 31);
        }
        l
    }

    #[test]
    fn the_key_is_the_nonce_words_and_the_crc() {
        let mut nonce = [0; 32];
        nonce[0] = 0x01;
        nonce[4] = 0x02;
        nonce[7] = 0x80;
        // Past the 28 bytes.
        nonce[28] = 0xff;
        assert_eq!(auth_key(&nonce, [0x10, 0x20]), 0x8000_2013);
    }

    #[test]
    fn rates() {
        assert_eq!(rate_registers(1_000_000), Some((0x64, 0xff)));
        assert_eq!(rate_registers(2_500_000), Some((0x28, 0xff)));
        assert_eq!(rate_registers(50_000_000), Some((0x02, 0x40)));
        assert_eq!(rate_registers(3_000_000), None);
    }

    #[test]
    fn the_converter_takes_channels_in_turn_first_sample_in_bit_31() {
        let mut c = Converter::new(0b101);
        // Channel 0 in the batch's first sample, channel 2 in its last.
        let mut data = 0x8000_0000u32.to_le_bytes().to_vec();
        data.extend_from_slice(&1u32.to_le_bytes());
        let out = c.convert(&data);
        assert_eq!(out.len(), 64);
        assert_eq!(out[..2], [1, 0]);
        assert_eq!(out[62..], [4, 0]);
        assert!(out[2..62].iter().all(|&b| b == 0));
    }

    #[test]
    fn a_partial_batch_and_a_partial_word_wait() {
        let mut c = Converter::new(0b11);
        let words: Vec<u8> = [0xffff_ffffu32, 0].iter().flat_map(|w| w.to_le_bytes()).collect();
        // A word and a half, then the rest: nothing until the batch is
        // whole.
        assert!(c.convert(&words[..6]).is_empty());
        let out = c.convert(&words[6..]);
        assert_eq!(out.len(), 64);
        assert!(out.chunks(2).all(|s| s == [1, 0]));
        // A short transfer gives only its own samples.
        let mut one = Converter::new(0b1);
        assert_eq!(one.convert(&[0; 4]).len(), 64);
        assert!(one.convert(&[]).is_empty());
    }

    #[test]
    fn no_channels_convert_to_nothing() {
        assert!(Converter::new(0).convert(&[0; 8]).is_empty());
    }

    #[test]
    fn firmware_parts_go_out_by_address() {
        let (mut l, r) = link(
            "ctl_out 40 a0 3000 4000 aabb -> ok\n\
             ctl_out 40 a0 3000 4000 - -> ok\n",
        );
        let mut image = vec![2, 0, 0x00, 0x30, 0x00, 0x40, 0, 0xaa, 0xbb];
        image.extend_from_slice(&[0, 0, 0x00, 0x30, 0x00, 0x40, 1]);
        send_firmware(l.transport(), &image).unwrap();
        r.finish().unwrap();
    }

    #[test]
    fn broken_firmware_files_are_refused() {
        // Over 4 KiB.
        let (mut l, _) = link("");
        assert!(send_firmware(l.transport(), &[1, 0x10, 0, 0, 0, 0, 0]).is_err());
        // A part the file does not hold.
        let (mut l, _) = link("");
        assert!(send_firmware(l.transport(), &[2, 0, 0, 0, 0, 0, 0, 0xaa]).is_err());
        // No part at all, a non-empty last part, an empty unflagged one.
        let (mut l, _) = link("");
        assert!(send_firmware(l.transport(), &[0, 0, 0]).is_err());
        let (mut l, r) = link("ctl_out 40 a0 0000 0000 aa -> ok\n");
        assert!(send_firmware(l.transport(), &[1, 0, 0, 0, 0, 0, 1, 0xaa]).is_err());
        r.finish().unwrap();
        let (mut l, r) = link("ctl_out 40 a0 0000 0000 - -> ok\n");
        assert!(send_firmware(l.transport(), &[0, 0, 0, 0, 0, 0, 0]).is_err());
        r.finish().unwrap();
    }

    #[test]
    fn the_wake_message_names_the_byte_it_checks() {
        // protocol.c:341 would print the count byte, 0x04.
        let (mut l, _) = link(
            "bulk_out 01 0089 -> 2\nbulk_in 81 1 -> 00\n\
             bulk_out 01 0088c00400 -> 5\nbulk_in 81 5 -> 0204003343\n",
        );
        let e = l.wake_i2c().unwrap_err().to_string();
        assert!(e.contains("I2C wake read (0x00)"), "{e}");
    }

    #[test]
    fn the_read_back_message_names_the_register() {
        // protocol.c:540 would print the init answer, 0x00.
        let (mut l, _) = link(
            "bulk_out 01 007f010012 -> 5\nbulk_in 81 1 -> 00\n\
             bulk_out 01 0080017faa -> 5\nbulk_in 81 1 -> 00\n\
             bulk_out 01 0081017f -> 4\nbulk_in 81 1 -> 55\n",
        );
        let e = l.send_bitstream(&[0x12]).unwrap_err().to_string();
        assert!(e.contains("(0x55 != 0xaa)"), "{e}");
    }

    #[test]
    fn a_reseed_has_no_answer_and_a_write_one_byte() {
        let (mut l, r) = link(
            "bulk_out 01 20244b358e -> 5\n\
             bulk_out 01 0080010080 -> 5\nbulk_in 81 1 -> 00\n",
        );
        l.reseed().unwrap();
        l.write_reg(0x00, 0x80).unwrap();
        r.finish().unwrap();
    }

    #[test]
    fn a_short_and_a_long_answer_fail() {
        let (mut l, _) = link(
            "bulk_out 01 0002 -> 2\nbulk_in 81 2 -> 00\n\
             bulk_out 01 0002 -> 2\nbulk_in 81 2 -> 000000\n",
        );
        let e = l.transact(&[0, 2], 2).unwrap_err().to_string();
        assert!(e.contains("incorrect length 1 != 2"), "{e}");
        let e = l.transact(&[0, 2], 2).unwrap_err().to_string();
        assert!(e.contains("OVERFLOW"), "{e}");
    }
}
