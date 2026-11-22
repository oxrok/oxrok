// Counterpart in sigrok:
//   libsigrok/src/hardware/wch-ch32h417-libusb/protocol.c,
//   libsigrok/src/hardware/wch-ch32h417-libusb/protocol.h,
//   libsigrok/src/hardware/wch-ch32h417-libusb/api.c (the boards, the threshold),
//   libsigrok/src/hardware/wch-ch32h417-libusb/libusb_wrapper.h (the sample transfers),
//   Copyright (C) 2026 WeAct Studio,
//   with commits by oxrok-bot[bot]
//   (libsigrok libsigrok-unreleased-1859-g4ed8c99c).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The wire protocol of the CH32H417 logic analyzer firmware, as the
//! `WeAct` tree's `wch-ch32h417-libusb` driver speaks it. The firmware is
//! closed; every fact here is that driver's (`protocol.c`, `protocol.h`,
//! `api.c`, `libusb_wrapper.c`).
//!
//! # The device
//!
//! `1a86:5537`, interface 0, three bulk pipes:
//!
//! | endpoint | what |
//! |---|---|
//! | 1 OUT | commands, always 64 bytes |
//! | 1 IN | answers, 32 bytes |
//! | 2 IN | logic samples |
//! | 3 IN | ADC samples |
//!
//! The serial number string tells the boards apart ([`Profile`]): one
//! starting `01` is the original board with an ADC, one starting `b5` the
//! `WeAct` Studio Logic Analyzer V2. Any other is not taken.
//!
//! # Commands
//!
//! A command is `[command, length, data...]`, zero padded to 64 bytes.
//! An answer is `[command | 0x10, length, status, data...]`, where the
//! length counts the status byte and the data, and a status other than 0
//! is a refusal. The start and stop commands have no answer.
//!
//! | command | data | answer's data |
//! |---|---|---|
//! | `a0` set logic parameters | width (`08`, `10`), rate index, key stop (0, 1) | none |
//! | `a1` set threshold | 16 bits, high byte first | none |
//! | `a2` start logic | none | no answer |
//! | `a3` stop logic | none | no answer |
//! | `a4` get logic parameters | none | width, rate index, key counter |
//! | `a5` get threshold | none | 1 byte (unused by the driver) |
//! | `a6` set ADC parameters | width (`08`, `0a`), rate index + 3 | none |
//! | `a7` set ADC channel | 0, 1 | none |
//! | `a8` start ADC | none | no answer |
//! | `a9` stop ADC | none | no answer |
//! | `aa` get ADC parameters | none | width, rate index (unused) |
//! | `ab` get ADC channel | none | 1 byte (unused) |
//! | `ac` get version | none | firmware version, hardware version |
//! | `ae` enter the boot loader | none | no answer (unused here) |
//! | `af` set LED brightness | percent | none |
//!
//! Firmware before the board's key was reported answers `a4` with two
//! bytes; the third says the firmware counts key presses, and the key
//! options exist only then.
//!
//! # Sample rates
//!
//! The rate goes down as its index in the table the host shows for the
//! link and the sample width; the firmware adds its own offset per USB
//! mode. So the tables are in the firmware's order and only ever grow at
//! the end ([`rates`]).
//!
//! # Samples
//!
//! After a start the device streams on endpoint 2: one byte a sample
//! with at most eight channels, two bytes little endian otherwise, with
//! no framing. The ADC streams on endpoint 3: a byte a sample at 8 bits,
//! two bytes little endian (10 bits used) at 10. A device whose link
//! cannot carry the rate stops sending on its own, and so does one whose
//! key ended the capture: the host tells the two apart by the key
//! counter.

use crate::core::{Error, Result};
use crate::usb::Transport;
use std::time::Duration;

/// The command and answer endpoints, and the two sample endpoints.
pub const EP_COMMAND: u8 = 0x01;
/// Answers.
pub const EP_ANSWER: u8 = 0x81;
/// Logic samples.
pub const EP_LOGIC: u8 = 0x82;
/// ADC samples.
pub const EP_ADC: u8 = 0x83;

/// A command is always this long (`PACKET_SIZE_OUT`).
pub const COMMAND_SIZE: usize = 64;
/// An answer is read as this many bytes (`PACKET_SIZE_IN`).
pub const ANSWER_SIZE: usize = 32;
/// `USB_TIMEOUT`, for commands and answers.
pub const TIMEOUT: Duration = Duration::from_millis(500);
/// `KEY_POLL_TIMEOUT_MS`: how long a key poll may wait.
pub const KEY_POLL_TIMEOUT: Duration = Duration::from_millis(20);
/// The timeout of the reads that drain the answer endpoint.
const FLUSH_TIMEOUT: Duration = Duration::from_millis(1);

/// `LIBUSB_STREAM_RING_CHUNK`: one sample transfer, 16 bursts of 16 KiB.
pub const STREAM_TRANSFER: usize = 256 * 1024;
/// `LIBUSB_STREAM_RING_COUNT`: sample transfers in flight.
pub const STREAM_DEPTH: usize = 32;
/// How long the samples may stay away before the device is taken to have
/// stopped (`no_data_timeout_ms`).
pub const SILENCE: Duration = Duration::from_millis(500);

/// Set the logic parameters: sample width, rate index, key stop.
pub const CMD_SET_LOGIC_PARAMS: u8 = 0xa0;
/// Set the input threshold.
pub const CMD_SET_LOGIC_LEVEL: u8 = 0xa1;
/// Start the logic stream; no answer.
pub const CMD_START_LOGIC: u8 = 0xa2;
/// Stop the logic stream; no answer.
pub const CMD_STOP_LOGIC: u8 = 0xa3;
/// Get the logic parameters and the key count.
pub const CMD_GET_LOGIC_PARAMS: u8 = 0xa4;
/// Set the ADC parameters: width, rate index.
pub const CMD_SET_ADC_PARAMS: u8 = 0xa6;
/// Select the ADC input.
pub const CMD_SET_ADC_CHANNEL: u8 = 0xa7;
/// Start the ADC stream; no answer.
pub const CMD_START_ADC: u8 = 0xa8;
/// Stop the ADC stream; no answer.
pub const CMD_STOP_ADC: u8 = 0xa9;
/// Get the firmware and hardware versions.
pub const CMD_GET_VERSION: u8 = 0xac;
/// Set the indicator brightness.
pub const CMD_SET_LED_BRIGHT: u8 = 0xaf;
/// An answer's command byte is the command's with this bit set.
pub const ANSWER_BIT: u8 = 0x10;

/// `ch32h417_libusb_logic_samplerates_usb3`.
pub const RATES_USB3: [u64; 25] = [
    200_000_000,
    187_500_000,
    175_000_000,
    162_500_000,
    156_250_000,
    150_000_000,
    137_500_000,
    125_000_000,
    112_500_000,
    100_000_000,
    93_750_000,
    87_500_000,
    81_250_000,
    78_125_000,
    75_000_000,
    68_750_000,
    50_000_000,
    40_000_000,
    30_000_000,
    20_000_000,
    16_000_000,
    12_000_000,
    8_000_000,
    6_000_000,
    4_000_000,
];
/// `..._usb2_16ch`: two bytes a sample over high speed.
pub const RATES_USB2_16: [u64; 6] =
    [20_000_000, 16_000_000, 12_000_000, 8_000_000, 6_000_000, 4_000_000];
/// `..._usb2_8ch`: one byte a sample over high speed.
pub const RATES_USB2_8: [u64; 8] =
    [40_000_000, 30_000_000, 20_000_000, 16_000_000, 12_000_000, 8_000_000, 6_000_000, 4_000_000];
/// `ch32h417_libusb_adc_samplerates`. `SR_MHZ(13.33)` and `SR_MHZ(11.43)`
/// are these integers.
pub const RATES_ADC: [u64; 5] = [20_000_000, 16_000_000, 13_330_000, 11_430_000, 10_000_000];
/// `CH32H417_ADC_DIVIDER_OFFSET`: an ADC rate's index goes down plus this.
pub const ADC_INDEX_OFFSET: u8 = 3;
/// The older boards show the USB 3 table without its last 11 entries
/// (`ch32h417_libusb_update_samplerate_list()`, "- 11").
const USB3_OLD_BOARD: usize = RATES_USB3.len() - 11;

/// What the device samples (`pattern`, as the C driver has it under
/// `SR_CONF_PATTERN_MODE`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Mode {
    /// The 16 logic inputs.
    Logic,
    /// One of the two ADC inputs.
    Adc,
}

impl Mode {
    /// The names `pattern` takes and lists, `mode_names[]`.
    pub const NAMES: [(&'static str, Mode); 2] =
        [("Logic Analyzer", Mode::Logic), ("ADC", Mode::Adc)];

    /// The name of the mode.
    pub fn name(self) -> &'static str {
        match self {
            Mode::Logic => "Logic Analyzer",
            Mode::Adc => "ADC",
        }
    }
}

/// A board the driver knows, by the start of its serial number
/// (`supported_devices[]`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Profile {
    /// The vendor string.
    pub vendor: &'static str,
    /// The model string.
    pub model: &'static str,
    /// What the serial number, in lower case, starts with.
    pub serial_start: &'static str,
    /// `DEV_CAPS_ADC`: two analog inputs and the ADC mode.
    pub adc: bool,
}

impl Profile {
    /// The original board: an ADC, the short rate tables, a threshold
    /// DAC taken in raw counts.
    pub const ORIGINAL: Profile = Profile {
        vendor: "CH32H417",
        model: "CH32H417 Logic Analyzer",
        serial_start: "01",
        adc: true,
    };
    /// The `WeAct` Studio Logic Analyzer V2.
    pub const WEACT_V2: Profile = Profile {
        vendor: "CH32H417",
        model: "WeAct Studio Logic Analyzer V2",
        serial_start: "b5",
        adc: false,
    };
    /// In the C table's order: the first whose start matches is taken.
    pub const ALL: [Profile; 2] = [Profile::ORIGINAL, Profile::WEACT_V2];

    /// The profile of a board with this serial number, already in lower
    /// case.
    pub fn of_serial(serial: &str) -> Option<Profile> {
        Profile::ALL.into_iter().find(|p| serial.starts_with(p.serial_start))
    }

    /// Whether the board has the full rate tables and the threshold in
    /// input volts: `str_startswith(devc->profile->SN_start, "b5")`.
    pub fn is_v2(self) -> bool {
        self.serial_start == "b5"
    }
}

/// The rates the device offers, in the order of their indexes
/// (`ch32h417_libusb_update_samplerate_list()`): by the mode, the link
/// and, over USB 2, the bytes a sample takes. An older board has one
/// rate over USB 2 and the first 14 over USB 3.
pub fn rates(profile: Profile, mode: Mode, usb2: bool, two_bytes: bool) -> &'static [u64] {
    let full = profile.is_v2();
    match (mode, usb2, two_bytes) {
        (Mode::Adc, _, _) => &RATES_ADC,
        (Mode::Logic, true, true) => &RATES_USB2_16[..if full { RATES_USB2_16.len() } else { 1 }],
        (Mode::Logic, true, false) => &RATES_USB2_8[..if full { RATES_USB2_8.len() } else { 1 }],
        (Mode::Logic, false, _) => {
            &RATES_USB3[..if full { RATES_USB3.len() } else { USB3_OLD_BOARD }]
        }
    }
}

/// `samplerate_to_divider()`: the index of `rate` in `table`.
pub fn rate_index(table: &[u64], rate: u64) -> Option<u8> {
    table.iter().position(|r| *r == rate).and_then(|i| u8::try_from(i).ok())
}

/// The thresholds `voltage_threshold` lists: the level on the input
/// pins, 0.6 V to 1.5 V in steps of 0.05 V on the V2 and 0.8 V to 2.3 V
/// in steps of 0.1 V on the original board.
pub fn threshold_range(profile: Profile) -> (f64, f64, f64) {
    if profile.is_v2() { (0.6, 1.5, 0.05) } else { (0.8, 2.3, 0.1) }
}

/// `voltage_to_raw[]`: tenths of a volt to the original board's DAC
/// counts.
const VOLTAGE_TO_RAW: [(u16, u16); 16] = [
    (23, 33),
    (22, 93),
    (21, 123),
    (20, 200),
    (19, 225),
    (18, 280),
    (17, 308),
    (16, 400),
    (15, 428),
    (14, 500),
    (13, 545),
    (12, 574),
    (11, 620),
    (10, 675),
    (9, 725),
    (8, 751),
];

/// The 16 bits `a1` takes for a threshold of `volts` on the pins
/// (`config_set()`, `SR_CONF_VOLTAGE_THRESHOLD`).
///
/// The V2 takes the level at its input in tenths of a volt with bit 15
/// set: 0.6 V on the pins is 1.0 V at the input and 1.5 V is 3.3 V, and
/// the levels between lie on that line. The original board takes DAC
/// counts from a table of tenths of a volt; a level the table does not
/// have goes down as the tenths themselves, as in C.
pub fn threshold_code(profile: Profile, volts: f64) -> Result<u16> {
    if profile.is_v2() {
        let input = 1.0 + (volts - 0.6) * (3.3 - 1.0) / (1.5 - 0.6);
        let code = (input * 10.0).round();
        if !(10.0..=33.0).contains(&code) {
            return Err(Error::Arg(format!(
                "voltage_threshold Input Level value out of range: {code:.0} (must be 10-33)"
            )));
        }
        return Ok(code as u16 | 0x8000);
    }
    let tenths = (volts * 10.0).round();
    // C casts the double to uint16_t, which is undefined out of range.
    if !(0.0..=1024.0).contains(&tenths) {
        return Err(Error::Arg(format!(
            "voltage_threshold DAC value out of range: {tenths:.0} (must be 0-1024)"
        )));
    }
    let tenths = tenths as u16;
    Ok(VOLTAGE_TO_RAW.iter().find(|(v, _)| *v == tenths).map_or(tenths, |(_, raw)| *raw))
}

/// An ADC sample in volts, as `adc_send_data_proc()` scales it: 3.3 V
/// full scale behind a 220:320 divider, in single precision.
pub fn adc_volts(data: &[u8], ten_bit: bool) -> Vec<f32> {
    if ten_bit {
        let scale = 320.0f32 / 220.0f32 / 1024.0f32 * 3.3f32;
        data.as_chunks::<2>()
            .0
            .iter()
            .map(|s| f32::from(u16::from_le_bytes(*s) & 0x3ff) * scale)
            .collect()
    } else {
        let scale = 320.0f32 / 220.0f32 / 256.0f32 * 3.3f32;
        data.iter().map(|s| f32::from(*s) * scale).collect()
    }
}

/// What `a4` answers.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LogicParams {
    /// The sample width, `0x08` or `0x10`.
    pub width: u8,
    /// The rate index.
    pub index: u8,
    /// The count of key presses, from firmware that reports it.
    pub key_seq: Option<u8>,
}

/// The command channel of an open, claimed device.
pub struct Link {
    t: Box<dyn Transport>,
    /// The timeout of commands and answers; lowered for a key poll.
    timeout: Duration,
    /// Whether the firmware counts key presses: the last `a4` answer had
    /// its third byte.
    pub key_available: bool,
    /// The key count the host has acted on (`last_key_seq`).
    last_key_seq: u8,
}

impl Link {
    /// A link on a transport whose interface 0 is claimed.
    pub fn new(t: Box<dyn Transport>) -> Self {
        Link { t, timeout: TIMEOUT, key_available: false, last_key_seq: 0 }
    }

    /// The transport, for the sample stream.
    pub fn transport(&mut self) -> &mut dyn Transport {
        self.t.as_mut()
    }

    /// `ch32h417_libusb_flush_cmd_endpoint()`: answers nobody read are
    /// read with a timeout of 1 ms and dropped, at most 11 of them.
    pub fn flush(&mut self) {
        std::thread::sleep(Duration::from_millis(1));
        for tries in 0.. {
            match self.t.bulk_in(EP_ANSWER, ANSWER_SIZE, FLUSH_TIMEOUT) {
                Ok(d) if !d.is_empty() => {
                    log::warn!("Device has data not read, ignore it. Data: {:02x}", d[0]);
                }
                // Nothing there, or a read that failed.
                _ => break,
            }
            if tries >= 10 {
                log::error!("Timeout while flushing command endpoint");
                break;
            }
            std::thread::sleep(Duration::from_millis(1));
        }
        std::thread::sleep(Duration::from_millis(1));
    }

    /// `ch32h417_libusb_send_command()`.
    pub fn command(&mut self, command: u8, data: &[u8]) -> Result<()> {
        let mut packet = [0u8; COMMAND_SIZE];
        packet[0] = command;
        packet[1] = data.len() as u8;
        packet[2..2 + data.len()].copy_from_slice(data);
        self.t.bulk_out(EP_COMMAND, &packet, self.timeout).map(|_| ()).inspect_err(|e| {
            log::debug!("Bulk write to pipe 1 failed: {e}");
            log::error!("Endpoint write failed for command 0x{command:02x}");
        })
    }

    /// `ch32h417_libusb_receive_response()`: the answer to `command`, its
    /// data cut to `capacity` bytes.
    pub fn answer(&mut self, command: u8, capacity: usize) -> Result<Vec<u8>> {
        let expected = command | ANSWER_BIT;
        let mut d = match self.t.bulk_in(EP_ANSWER, ANSWER_SIZE, self.timeout) {
            Ok(d) => d,
            // No data is no error to the C read, and a short answer next.
            Err(Error::Timeout) => Vec::new(),
            Err(e) => {
                log::debug!("Bulk read from pipe 1 failed: {e}");
                log::error!("Endpoint read failed for response");
                return Err(e);
            }
        };
        d.truncate(ANSWER_SIZE);
        if d.len() < 2 {
            log::error!("Response too short: {} bytes", d.len());
            return Err(Error::device(format!(
                "no answer to command 0x{command:02x} ({} bytes)",
                d.len()
            )));
        }
        if d[0] != expected {
            log::error!("Unexpected response: 0x{:02x} (expected 0x{expected:02x})", d[0]);
            return Err(Error::device(format!(
                "answer 0x{:02x} to command 0x{command:02x}, expected 0x{expected:02x}",
                d[0]
            )));
        }
        let declared = usize::from(d[1]);
        if declared > 0 {
            if d.len() < 3 {
                log::error!("Response 0x{expected:02x} has no status byte ({} bytes)", d.len());
                return Err(Error::device(format!("answer 0x{expected:02x} without its status")));
            }
            if d[2] != 0 {
                log::error!("Command failed with status 0x{:02x}", d[2]);
                return Err(Error::device(format!(
                    "the device refused command 0x{command:02x} with status 0x{:02x}",
                    d[2]
                )));
            }
        }
        let mut payload = declared.saturating_sub(1);
        let available = d.len().saturating_sub(3);
        if payload > available {
            log::warn!(
                "Response 0x{expected:02x} declares {payload} data bytes, only {available} received"
            );
            payload = available;
        }
        if payload > capacity {
            // C says so only when the caller gave a buffer.
            if capacity > 0 {
                log::warn!(
                    "Response 0x{expected:02x} truncated: {payload} data bytes > buffer {capacity}"
                );
            }
            payload = capacity;
        }
        Ok(d[3..3 + payload].to_vec())
    }

    /// A command with an answer that carries no data.
    fn set(&mut self, command: u8, data: &[u8]) -> Result<()> {
        self.command(command, data)?;
        self.answer(command, 0).map(|_| ())
    }

    /// `ch32h417_libusb_set_logic_params()`.
    pub fn set_logic_params(&mut self, two_bytes: bool, index: u8, key_stop: bool) -> Result<()> {
        let width = if two_bytes { 0x10 } else { 0x08 };
        log::info!(
            "Setting logic params: width=0x{width:02x}, div={index}, key_stop={}",
            u8::from(key_stop)
        );
        self.set(CMD_SET_LOGIC_PARAMS, &[width, index, u8::from(key_stop)])
    }

    /// `ch32h417_libusb_get_logic_params()`, which also learns whether
    /// the firmware reports the key.
    pub fn logic_params(&mut self) -> Result<LogicParams> {
        self.command(CMD_GET_LOGIC_PARAMS, &[])?;
        let d = self.answer(CMD_GET_LOGIC_PARAMS, 3)?;
        self.key_available = d.len() >= 3;
        let at = |i: usize| d.get(i).copied().unwrap_or(0);
        Ok(LogicParams { width: at(0), index: at(1), key_seq: d.get(2).copied() })
    }

    /// The first key count, taken at the open so that presses from before
    /// it are not read as new (`dev_open()`).
    pub fn learn_key(&mut self) {
        if let Ok(p) = self.logic_params() {
            self.last_key_seq = p.key_seq.unwrap_or(0);
        }
    }

    /// `ch32h417_libusb_key_seq_changed()`: whether the key was pressed
    /// since this was last asked. Never on firmware without the count, and
    /// a device that does not answer within 20 ms counts as no press.
    pub fn key_pressed(&mut self) -> bool {
        if !self.key_available {
            return false;
        }
        self.timeout = KEY_POLL_TIMEOUT;
        let r = self.logic_params();
        self.timeout = TIMEOUT;
        let Ok(p) = r else { return false };
        let seq = p.key_seq.unwrap_or(0);
        if seq == self.last_key_seq {
            return false;
        }
        self.last_key_seq = seq;
        true
    }

    /// `ch32h417_libusb_set_threshold_value()`.
    pub fn set_threshold(&mut self, code: u16) -> Result<()> {
        log::info!("Setting DAC threshold: 0x{code:04x} ({code})");
        self.set(CMD_SET_LOGIC_LEVEL, &code.to_be_bytes())
    }

    /// `ch32h417_libusb_set_led_brightness()`, in percent.
    pub fn set_led_brightness(&mut self, percent: u8) -> Result<()> {
        log::info!("Setting indicator brightness: {percent}%");
        self.set(CMD_SET_LED_BRIGHT, &[percent.min(100)])
    }

    /// `ch32h417_libusb_get_fw_version()`: the firmware version in the
    /// high byte and the hardware version in the low one, 0 for an answer
    /// with no data.
    pub fn version(&mut self) -> Result<u16> {
        self.command(CMD_GET_VERSION, &[]).inspect_err(|_| {
            log::error!("Failed to send get version command");
        })?;
        let d = self.answer(CMD_GET_VERSION, 2).inspect_err(|_| {
            log::error!("Failed to receive version response");
        })?;
        let Some(&firmware) = d.first() else {
            log::warn!("Version response too short: 0");
            return Ok(0);
        };
        let hardware = d.get(1).copied().unwrap_or(0);
        log::info!("Firmware version: {firmware}, Hardware version: {hardware}");
        Ok(u16::from(firmware) << 8 | u16::from(hardware))
    }

    /// `ch32h417_libusb_set_adc_params()`.
    pub fn set_adc_params(&mut self, ten_bit: bool, index: u8) -> Result<()> {
        let width = if ten_bit { 0x0a } else { 0x08 };
        let index = index + ADC_INDEX_OFFSET;
        log::info!("Setting ADC params: width=0x{width:02x}, div={index}");
        self.set(CMD_SET_ADC_PARAMS, &[width, index])
    }

    /// `ch32h417_libusb_set_adc_channel()`.
    pub fn set_adc_channel(&mut self, channel: u8) -> Result<()> {
        log::info!("Setting ADC channel: {channel}");
        self.set(CMD_SET_ADC_CHANNEL, &[channel])
    }

    /// The start command of `mode`, which has no answer.
    pub fn start(&mut self, mode: Mode) -> Result<()> {
        match mode {
            Mode::Logic => {
                log::info!("Starting logic acquisition");
                self.command(CMD_START_LOGIC, &[])
            }
            Mode::Adc => {
                log::info!("Starting ADC acquisition");
                self.command(CMD_START_ADC, &[])
            }
        }
    }

    /// The stop command of `mode`, which has no answer.
    pub fn stop(&mut self, mode: Mode) -> Result<()> {
        match mode {
            Mode::Logic => {
                log::info!("Stopping logic acquisition");
                self.command(CMD_STOP_LOGIC, &[])
            }
            Mode::Adc => {
                log::info!("Stopping ADC acquisition");
                self.command(CMD_STOP_ADC, &[])
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn profiles_go_by_the_serial_numbers_start() {
        assert_eq!(Profile::of_serial("b5e339e33906e19c7f6416064c"), Some(Profile::WEACT_V2));
        assert_eq!(Profile::of_serial("0123"), Some(Profile::ORIGINAL));
        assert_eq!(Profile::of_serial("000000000000"), None);
        assert_eq!(Profile::of_serial(""), None);
        // The scan lowers the case before it compares.
        assert_eq!(Profile::of_serial("B5E3"), None);
        assert!(Profile::WEACT_V2.is_v2() && !Profile::WEACT_V2.adc);
        assert!(!Profile::ORIGINAL.is_v2() && Profile::ORIGINAL.adc);
    }

    #[test]
    fn the_rate_tables_follow_the_board_the_link_and_the_width() {
        let v2 = Profile::WEACT_V2;
        let old = Profile::ORIGINAL;
        assert_eq!(rates(v2, Mode::Logic, false, true), &RATES_USB3[..]);
        assert_eq!(rates(v2, Mode::Logic, false, false), &RATES_USB3[..]);
        assert_eq!(rates(v2, Mode::Logic, true, true), &RATES_USB2_16[..]);
        assert_eq!(rates(v2, Mode::Logic, true, false), &RATES_USB2_8[..]);
        assert_eq!(rates(old, Mode::Logic, false, true).len(), 14);
        assert_eq!(rates(old, Mode::Logic, false, true).last(), Some(&78_125_000));
        assert_eq!(rates(old, Mode::Logic, true, true), &[20_000_000]);
        assert_eq!(rates(old, Mode::Logic, true, false), &[40_000_000]);
        assert_eq!(rates(old, Mode::Adc, true, false), &RATES_ADC[..]);
        // The index is the place in the table shown, not in the USB 3 one.
        assert_eq!(rate_index(&RATES_USB3, 4_000_000), Some(24));
        assert_eq!(rate_index(&RATES_USB2_16, 4_000_000), Some(5));
        assert_eq!(rate_index(&RATES_USB2_8, 40_000_000), Some(0));
        assert_eq!(rate_index(&RATES_USB2_16, 40_000_000), None);
        assert_eq!(rate_index(&RATES_ADC, 13_330_000), Some(2));
    }

    #[test]
    fn the_v2_threshold_is_the_input_level_in_tenths() {
        let v2 = Profile::WEACT_V2;
        assert_eq!(threshold_code(v2, 0.6).unwrap(), 0x800a);
        assert_eq!(threshold_code(v2, 1.5).unwrap(), 0x8021);
        assert_eq!(threshold_code(v2, 1.0).unwrap(), 0x8014);
        assert!(threshold_code(v2, 0.5).is_err());
        assert!(threshold_code(v2, 1.6).is_err());
        assert!(threshold_code(v2, 0.0).is_err());
        assert_eq!(threshold_range(v2), (0.6, 1.5, 0.05));
    }

    #[test]
    fn the_original_boards_threshold_is_a_dac_count() {
        let old = Profile::ORIGINAL;
        assert_eq!(threshold_code(old, 2.3).unwrap(), 33);
        assert_eq!(threshold_code(old, 0.8).unwrap(), 751);
        assert_eq!(threshold_code(old, 1.5).unwrap(), 428);
        // Not in the table: the tenths go down as they are.
        assert_eq!(threshold_code(old, 0.0).unwrap(), 0);
        assert_eq!(threshold_code(old, 3.3).unwrap(), 33);
        assert!(threshold_code(old, -1.0).is_err());
        assert!(threshold_code(old, 200.0).is_err());
        assert_eq!(threshold_range(old), (0.8, 2.3, 0.1));
    }

    #[test]
    fn adc_samples_scale_to_volts() {
        let full8 = 320.0f32 / 220.0 / 256.0 * 3.3;
        assert_eq!(adc_volts(&[0, 1, 255], false), vec![0.0, full8, 255.0 * full8]);
        let full10 = 320.0f32 / 220.0 / 1024.0 * 3.3;
        // Little endian, ten bits, and half a sample at the end dropped.
        assert_eq!(adc_volts(&[0x01, 0x00, 0xff, 0xff, 0x07], true), vec![full10, 1023.0 * full10]);
    }
}
