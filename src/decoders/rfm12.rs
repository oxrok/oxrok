// Counterpart in sigrok:
//   libsigrokdecode/decoders/rfm12/pd.py,
//   Copyright (C) 2014 Sławek Piotrowski <sentinel@atteo.org>,
//   GPL-2.0-or-later.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the RFM12 command set and the decoder's documented
//   classes and texts (including the positional bit annotations).

//! `HopeRF` RFM12 wireless transceiver control protocol on an SPI bus.

// The handlers keep pd.py's statement order and its local tables, so
// the lint's preferred restructurings would obscure the correspondence.
#![allow(clippy::if_not_else, clippy::items_after_statements)]

use crate::core::{Error, Result};
use crate::decode::{AnnClass, AnnRow, Config, Context, Decoder, DecoderDef, DecoderInfo, Item};

// Annotation class indices, as the libsigrokdecode decoder numbers them.
const ANN_CMD: usize = 0;
const ANN_PARAMS: usize = 1;
const ANN_DISABLED: usize = 2;
const ANN_RETURN: usize = 3;
const ANN_DISABLED_RETURN: usize = 4;
const ANN_INTERPRETATION: usize = 5;

/// pd.py's `ann_to_row`.
const ANN_TO_ROW: [usize; 6] = [0, 0, 0, 1, 1, 2];

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "rfm12",
    name: "RFM12",
    longname: "HopeRF RFM12",
    desc: "HopeRF RFM12 wireless transceiver control protocol.",
    license: "gplv2+",
    inputs: &["spi"],
    outputs: &[],
    tags: &["Wireless/RF"],
    channels: &[],
    optional_channels: &[],
    options: &[],
    annotations: &[
        AnnClass { id: "cmd", desc: "Command" },
        AnnClass { id: "params", desc: "Command parameters" },
        AnnClass { id: "disabled", desc: "Disabled bits" },
        AnnClass { id: "return", desc: "Returned values" },
        AnnClass { id: "disabled_return", desc: "Disabled returned values" },
        AnnClass { id: "interpretation", desc: "Interpretation" },
    ],
    annotation_rows: &[
        AnnRow { id: "commands", desc: "Commands", classes: &[0, 1, 2] },
        AnnRow { id: "return", desc: "Return", classes: &[3, 4] },
        AnnRow { id: "interpretation", desc: "Interpretation", classes: &[5] },
    ],
    binary: &[],
    doc: "This decoder stacks on top of the 'spi' PD and decodes the HopeRF RFM12\n\
          wireless transceiver control protocol.",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(Rfm12::default())
}

/// One clocked bit: value, start sample, end sample.
type Bit = (i64, i64, i64);

/// The RFM12 decoder.
pub struct Rfm12 {
    mosi_bytes: Vec<i64>,
    miso_bytes: Vec<i64>,
    /// Bits in clock order, the first one clocked (MSB) first.
    mosi_bits: Vec<Bit>,
    row_pos: [i64; 3],

    // Initialize with Power-On-Reset values.
    last_status: [i64; 2],
    last_config: i64,
    last_power: i64,
    last_freq: i64,
    last_data_rate: i64,
    last_fifo_and_reset: i64,
    last_afc: i64,
    last_transceiver: i64,
    last_pll: i64,

    /// Set when pd.py would have raised and ended the decoder.
    dead: bool,
}

impl Default for Rfm12 {
    fn default() -> Self {
        Rfm12 {
            mosi_bytes: Vec::new(),
            miso_bytes: Vec::new(),
            mosi_bits: Vec::new(),
            row_pos: [0, 8, 8],
            last_status: [0x00, 0x00],
            last_config: 0x08,
            last_power: 0x08,
            last_freq: 0x680,
            last_data_rate: 0x23,
            last_fifo_and_reset: 0x80,
            last_afc: 0xF7,
            last_transceiver: 0x00,
            last_pll: 0x77,
            dead: false,
        }
    }
}

impl Rfm12 {
    /// Python's list indexing, negative indices included.
    fn bit_at(bits: &[Bit], index: i64) -> Result<Bit> {
        let index = if index < 0 { bits.len() as i64 + index } else { index };
        usize::try_from(index)
            .ok()
            .and_then(|i| bits.get(i).copied())
            .ok_or_else(|| Error::arg("rfm12: bit position outside the word"))
    }

    fn advance_ann(&mut self, ann: usize, length: i64) {
        self.row_pos[ANN_TO_ROW[ann]] += length;
    }

    /// pd.py's `putx`: places `length` bits at the current position of
    /// the annotation's row and advances it.
    fn putx(
        &mut self,
        ctx: &mut dyn Context,
        ann: usize,
        length: i64,
        description: Vec<String>,
    ) -> Result<()> {
        let row = ANN_TO_ROW[ann];
        let bit = self.row_pos[row];
        let first = Self::bit_at(&self.mosi_bits, bit)?;
        let last = Self::bit_at(&self.mosi_bits, bit + length - 1)?;
        ctx.put_ann(first.1 as u64, last.2 as u64, ann, description);
        self.row_pos[row] = bit + length;
        Ok(())
    }

    /// pd.py's `describe_bits`, one annotation per named bit.
    fn describe_bits(&mut self, ctx: &mut dyn Context, data: i64, names: &[&[&str]]) -> Result<()> {
        let mut i = 1i64 << (names.len() - 1);
        let mut bit = 0;
        while i != 0 {
            if !names[bit].is_empty() {
                let ann = if data & i != 0 { ANN_PARAMS } else { ANN_DISABLED };
                let texts = names[bit].iter().map(|s| (*s).to_owned()).collect();
                self.putx(ctx, ann, 1, texts)?;
            }
            i >>= 1;
            bit += 1;
        }
        Ok(())
    }

    /// pd.py's `describe_return_bits`.
    fn describe_return_bits(
        &mut self,
        ctx: &mut dyn Context,
        data: i64,
        names: &[&str],
    ) -> Result<()> {
        let mut i = 1i64 << (names.len() - 1);
        let mut bit = 0;
        while i != 0 {
            if !names[bit].is_empty() {
                let ann = if data & i != 0 { ANN_RETURN } else { ANN_DISABLED_RETURN };
                self.putx(ctx, ann, 1, vec![names[bit].to_owned()])?;
            } else {
                self.advance_ann(ANN_RETURN, 1);
            }
            i >>= 1;
            bit += 1;
        }
        Ok(())
    }

    /// pd.py's `describe_changed_bits`.
    fn describe_changed_bits(
        &mut self,
        ctx: &mut dyn Context,
        data: i64,
        old_data: i64,
        names: &[&[&str]],
    ) -> Result<()> {
        let changes = data ^ old_data;
        let mut i = 1i64 << (names.len() - 1);
        let mut bit = 0;
        while i != 0 {
            if !names[bit].is_empty() && changes & i != 0 {
                let texts = if data & i != 0 {
                    vec!["+".to_owned(), "Turning on".to_owned()]
                } else {
                    vec!["-".to_owned(), "Turning off".to_owned()]
                };
                self.putx(ctx, ANN_INTERPRETATION, 1, texts)?;
            } else {
                self.advance_ann(ANN_INTERPRETATION, 1);
            }
            i >>= 1;
            bit += 1;
        }
        Ok(())
    }

    fn handle_configuration_cmd(&mut self, ctx: &mut dyn Context, cmd: &[i64]) -> Result<()> {
        self.putx(
            ctx,
            ANN_CMD,
            8,
            vec!["Configuration command".to_owned(), "Configuration".to_owned()],
        )?;
        let names: [&[&str]; 2] = [&["Internal data register", "el"], &["FIFO mode", "ef"]];

        let bits = (cmd[1] & 0xC0) >> 6;
        let old_bits = (self.last_config & 0xC0) >> 6;
        self.describe_bits(ctx, bits, &names)?;
        self.describe_changed_bits(ctx, bits, old_bits, &names)?;

        const FREQUENCIES: [&str; 4] = ["315", "433", "868", "915"];
        let f = format!("{}MHz", FREQUENCIES[((cmd[1] & 0x30) >> 4) as usize]);
        self.putx(ctx, ANN_PARAMS, 2, vec![format!("Frequency: {f}"), f])?;
        if cmd[1] & 0x30 != self.last_config & 0x30 {
            self.putx(ctx, ANN_INTERPRETATION, 2, vec!["Changed".to_owned(), "~".to_owned()])?;
        }

        let c = format!("{:.1}pF", 8.5 + (cmd[1] & 0xF) as f64 * 0.5);
        self.putx(ctx, ANN_PARAMS, 4, vec![format!("Capacitance: {c}"), c])?;
        if cmd[1] & 0xF != self.last_config & 0xF {
            self.putx(ctx, ANN_INTERPRETATION, 4, vec!["Changed".to_owned(), "~".to_owned()])?;
        }

        self.last_config = cmd[1];
        Ok(())
    }

    fn handle_power_management_cmd(&mut self, ctx: &mut dyn Context, cmd: &[i64]) -> Result<()> {
        self.putx(ctx, ANN_CMD, 8, vec!["Power management".to_owned(), "Power".to_owned()])?;
        let names: [&[&str]; 8] = [
            &["Receiver chain", "er"],
            &["Baseband circuit", "ebb"],
            &["Transmission", "et"],
            &["Synthesizer", "es"],
            &["Crystal oscillator", "ex"],
            &["Low battery detector", "eb"],
            &["Wake-up timer", "ew"],
            &["Clock output off switch", "dc"],
        ];

        self.describe_bits(ctx, cmd[1], &names)?;

        let mut power = cmd[1];

        // Some bits imply other, even if they are set to 0.
        if power & 0x80 != 0 {
            power |= 0x58;
        }
        if power & 0x20 != 0 {
            power |= 0x18;
        }
        self.describe_changed_bits(ctx, power, self.last_power, &names)?;

        self.last_power = power;
        Ok(())
    }

    fn handle_frequency_setting_cmd(&mut self, ctx: &mut dyn Context, cmd: &[i64]) -> Result<()> {
        self.putx(ctx, ANN_CMD, 4, vec!["Frequency setting".to_owned(), "Frequency".to_owned()])?;
        let f = ((cmd[1] & 0xF) << 8) + cmd[2];
        self.putx(ctx, ANN_CMD, 12, vec![format!("F = {:3.4}", f as f64)])?;
        self.row_pos[2] -= 4;
        if self.last_freq != f {
            self.putx(ctx, ANN_INTERPRETATION, 12, vec!["Changing".to_owned(), "~".to_owned()])?;
        }
        self.last_freq = f;
        Ok(())
    }

    fn handle_data_rate_cmd(&mut self, ctx: &mut dyn Context, cmd: &[i64]) -> Result<()> {
        self.putx(ctx, ANN_CMD, 8, vec!["Data rate command".to_owned(), "Data rate".to_owned()])?;
        let r = cmd[1] & 0x7F;
        let cs = (cmd[1] & 0x80) >> 7;
        let rate = 10000.0 / 29.0 / (r as f64 + 1.0) / (1.0 + 7.0 * cs as f64);
        self.putx(ctx, ANN_CMD, 8, vec![format!("{rate:3.1}kbps")])?;
        if self.last_data_rate != cmd[1] {
            self.putx(ctx, ANN_INTERPRETATION, 8, vec!["Changing".to_owned(), "~".to_owned()])?;
        }
        self.last_data_rate = cmd[1];
        Ok(())
    }

    fn handle_receiver_control_cmd(&mut self, ctx: &mut dyn Context, cmd: &[i64]) -> Result<()> {
        self.putx(ctx, ANN_CMD, 5, vec!["Receiver control command".to_owned()])?;
        let s = if cmd[0] & 0x04 != 0 { "interrupt input" } else { "VDI output" };
        self.putx(ctx, ANN_CMD, 1, vec![format!("pin16 = {s}")])?;
        const VDI_NAMES: [&str; 4] = ["Fast", "Medium", "Slow", "Always on"];
        let vdi_speed = VDI_NAMES[(cmd[0] & 0x3) as usize];
        self.putx(ctx, ANN_CMD, 2, vec![format!("VDI: {vdi_speed}")])?;
        const BANDWIDTH_NAMES: [&str; 8] =
            ["Reserved", "400kHz", "340kHz", "270kHz", "200kHz", "134kHz", "67kHz", "Reserved"];
        let bandwidth = BANDWIDTH_NAMES[((cmd[1] & 0xE0) >> 5) as usize];
        self.putx(ctx, ANN_CMD, 3, vec![format!("Bandwidth: {bandwidth}")])?;
        const LNA_GAIN_NAMES: [i64; 4] = [0, -6, -14, -20];
        let lna_gain = LNA_GAIN_NAMES[((cmd[1] & 0x18) >> 3) as usize];
        self.putx(ctx, ANN_CMD, 2, vec![format!("LNA gain: {lna_gain}dB")])?;
        const RSSI_THRESHOLD_NAMES: [&str; 8] =
            ["-103", "-97", "-91", "-85", "-79", "-73", "Reserved", "Reserved"];
        let rssi_threshold = RSSI_THRESHOLD_NAMES[(cmd[1] & 0x7) as usize];
        self.putx(ctx, ANN_CMD, 3, vec![format!("RSSI threshold: {rssi_threshold}")])?;
        Ok(())
    }

    fn handle_data_filter_cmd(&mut self, ctx: &mut dyn Context, cmd: &[i64]) -> Result<()> {
        self.putx(ctx, ANN_CMD, 8, vec!["Data filter command".to_owned()])?;
        let clock_recovery = if cmd[1] & 0x80 != 0 {
            "auto"
        } else if cmd[1] & 0x40 != 0 {
            "fast"
        } else {
            "slow"
        };
        self.putx(ctx, ANN_CMD, 2, vec![format!("Clock recovery: {clock_recovery} mode")])?;
        // Should always be 1.
        self.advance_ann(ANN_CMD, 1);
        let s = if cmd[1] & 0x10 != 0 { "analog" } else { "digital" };
        self.putx(ctx, ANN_CMD, 1, vec![format!("Data filter: {s}")])?;
        // Should always be 1.
        self.advance_ann(ANN_CMD, 1);
        self.putx(ctx, ANN_CMD, 3, vec![format!("DQD threshold: {}", cmd[1] & 0x7)])?;
        Ok(())
    }

    fn handle_fifo_and_reset_cmd(&mut self, ctx: &mut dyn Context, cmd: &[i64]) -> Result<()> {
        self.putx(ctx, ANN_CMD, 8, vec!["FIFO and reset command".to_owned()])?;
        let fifo_level = (cmd[1] & 0xF0) >> 4;
        self.putx(ctx, ANN_CMD, 4, vec![format!("FIFO trigger level: {fifo_level}")])?;
        let last_fifo_level = (self.last_fifo_and_reset & 0xF0) >> 4;
        if fifo_level != last_fifo_level {
            self.putx(ctx, ANN_INTERPRETATION, 4, vec!["Changing".to_owned(), "~".to_owned()])?;
        } else {
            self.advance_ann(ANN_INTERPRETATION, 4);
        }
        let s = if cmd[1] & 0x08 != 0 { "one byte" } else { "two bytes" };
        self.putx(ctx, ANN_CMD, 1, vec![format!("Synchron length: {s}")])?;
        if (cmd[1] & 0x08) != (self.last_fifo_and_reset & 0x08) {
            self.putx(ctx, ANN_INTERPRETATION, 1, vec!["Changing".to_owned(), "~".to_owned()])?;
        } else {
            self.advance_ann(ANN_INTERPRETATION, 1);
        }

        let fifo_fill = if cmd[1] & 0x04 != 0 {
            "Always"
        } else if cmd[1] & 0x02 != 0 {
            "After synchron pattern"
        } else {
            "Never"
        };
        self.putx(ctx, ANN_CMD, 2, vec![format!("FIFO fill: {fifo_fill}")])?;
        if (cmd[1] & 0x06) != (self.last_fifo_and_reset & 0x06) {
            self.putx(ctx, ANN_INTERPRETATION, 2, vec!["Changing".to_owned(), "~".to_owned()])?;
        } else {
            self.advance_ann(ANN_INTERPRETATION, 2);
        }

        let s = if cmd[1] & 0x01 != 0 { "non-sensitive" } else { "sensitive" };
        self.putx(ctx, ANN_CMD, 1, vec![format!("Reset mode: {s}")])?;
        if (cmd[1] & 0x01) != (self.last_fifo_and_reset & 0x01) {
            self.putx(ctx, ANN_INTERPRETATION, 1, vec!["Changing".to_owned(), "~".to_owned()])?;
        } else {
            self.advance_ann(ANN_INTERPRETATION, 1);
        }

        self.last_fifo_and_reset = cmd[1];
        Ok(())
    }

    fn handle_synchron_pattern_cmd(&mut self, ctx: &mut dyn Context, _cmd: &[i64]) -> Result<()> {
        self.putx(ctx, ANN_CMD, 8, vec!["Synchron pattern command".to_owned()])?;
        // pd.py then reads an undefined module global `pattern` and ends
        // with a NameError; no further packet is ever decoded.
        self.dead = true;
        Ok(())
    }

    fn handle_fifo_read_cmd(
        &mut self,
        ctx: &mut dyn Context,
        _cmd: &[i64],
        ret: &[i64],
    ) -> Result<()> {
        self.putx(ctx, ANN_CMD, 8, vec!["FIFO read command".to_owned(), "FIFO read".to_owned()])?;
        self.putx(ctx, ANN_RETURN, 8, vec![format!("Data: {:02X}", ret[1])])?;
        Ok(())
    }

    fn handle_afc_cmd(&mut self, ctx: &mut dyn Context, cmd: &[i64]) -> Result<()> {
        self.putx(ctx, ANN_CMD, 8, vec!["AFC command".to_owned()])?;
        const MODES: [&str; 4] = ["Off", "Once", "During receiving", "Always"];
        let mode = (cmd[1] & 0xC0) >> 6;
        self.putx(ctx, ANN_CMD, 2, vec![format!("Mode: {}", MODES[mode as usize])])?;
        if (cmd[1] & 0xC0) != (self.last_afc & 0xC0) {
            self.putx(ctx, ANN_INTERPRETATION, 2, vec!["Changing".to_owned(), "~".to_owned()])?;
        } else {
            self.advance_ann(ANN_INTERPRETATION, 2);
        }

        let range_limit = (cmd[1] & 0x30) >> 4;
        const FREQ_TABLE: [f64; 4] = [0.0, 2.5, 5.0, 7.5];
        let freq_delta = FREQ_TABLE[((self.last_config & 0x30) >> 4) as usize];

        if range_limit == 0 {
            self.putx(ctx, ANN_CMD, 2, vec!["Range: No limit".to_owned()])?;
        } else if range_limit == 1 {
            self.putx(
                ctx,
                ANN_CMD,
                2,
                vec![format!("Range: +/-{}kHz", (15.0 * freq_delta) as i64)],
            )?;
        } else if range_limit == 2 {
            self.putx(
                ctx,
                ANN_CMD,
                2,
                vec![format!("Range: +/-{}kHz", (7.0 * freq_delta) as i64)],
            )?;
        } else if range_limit == 3 {
            self.putx(
                ctx,
                ANN_CMD,
                2,
                vec![format!("Range: +/-{}kHz", (3.0 * freq_delta) as i64)],
            )?;
        }

        if (cmd[1] & 0x30) != (self.last_afc & 0x30) {
            self.putx(ctx, ANN_INTERPRETATION, 2, vec!["Changing".to_owned(), "~".to_owned()])?;
        } else {
            self.advance_ann(ANN_INTERPRETATION, 2);
        }

        let names: [&[&str]; 4] = [
            &["Strobe edge"],
            &["High accuracy mode"],
            &["Enable offset register"],
            &["Enable offset calculation"],
        ];
        self.describe_bits(ctx, cmd[1] & 0xF, &names)?;
        self.describe_changed_bits(ctx, cmd[1] & 0xF, self.last_afc & 0xF, &names)?;

        self.last_afc = cmd[1];
        Ok(())
    }

    fn handle_transceiver_control_cmd(&mut self, ctx: &mut dyn Context, cmd: &[i64]) -> Result<()> {
        self.putx(ctx, ANN_CMD, 8, vec!["Transceiver control command".to_owned()])?;
        self.putx(
            ctx,
            ANN_CMD,
            4,
            vec![format!("FSK frequency delta: {}kHz", 15 * ((cmd[1] & 0xF0) >> 4))],
        )?;
        if cmd[1] & 0xF0 != self.last_transceiver & 0xF0 {
            self.putx(ctx, ANN_INTERPRETATION, 4, vec!["Changing".to_owned(), "~".to_owned()])?;
        } else {
            self.advance_ann(ANN_INTERPRETATION, 4);
        }

        self.advance_ann(ANN_CMD, 1);
        self.advance_ann(ANN_INTERPRETATION, 1);
        // pd.py's format string is 'Relative power: %dB', so the
        // conversion consumes the "d" and the unit prints as "B".
        self.putx(ctx, ANN_CMD, 3, vec![format!("Relative power: {}B", cmd[1] & 0x07)])?;
        if (cmd[1] & 0x07) != (self.last_transceiver & 0x07) {
            self.putx(ctx, ANN_INTERPRETATION, 3, vec!["Changing".to_owned(), "~".to_owned()])?;
        } else {
            self.advance_ann(ANN_INTERPRETATION, 3);
        }
        self.last_transceiver = cmd[1];
        Ok(())
    }

    fn handle_pll_setting_cmd(&mut self, ctx: &mut dyn Context, cmd: &[i64]) -> Result<()> {
        self.putx(ctx, ANN_CMD, 8, vec!["PLL setting command".to_owned()])?;
        self.advance_ann(ANN_CMD, 1);
        self.putx(ctx, ANN_CMD, 2, vec!["Clock buffer rise and fall time".to_owned()])?;
        self.advance_ann(ANN_CMD, 1);
        self.advance_ann(ANN_INTERPRETATION, 4);
        let names: [&[&str]; 2] =
            [&["Delay in phase detector", "dly"], &["Disable dithering", "ddit"]];
        self.describe_bits(ctx, (cmd[1] & 0xC) >> 2, &names)?;
        self.describe_changed_bits(ctx, (cmd[1] & 0xC) >> 2, (self.last_pll & 0xC) >> 2, &names)?;
        let s = if cmd[1] & 0x01 != 0 { "256kbps, high" } else { "86.2kbps, low" };
        self.putx(ctx, ANN_CMD, 1, vec![format!("Max bit rate: {s} noise")])?;

        self.advance_ann(ANN_INTERPRETATION, 1);
        if (cmd[1] & 0x01) != (self.last_pll & 0x01) {
            self.putx(ctx, ANN_INTERPRETATION, 1, vec!["Changing".to_owned(), "~".to_owned()])?;
        }

        self.last_pll = cmd[1];
        Ok(())
    }

    fn handle_transmitter_register_cmd(
        &mut self,
        ctx: &mut dyn Context,
        cmd: &[i64],
    ) -> Result<()> {
        self.putx(
            ctx,
            ANN_CMD,
            8,
            vec!["Transmitter register command".to_owned(), "Transmit".to_owned()],
        )?;
        self.putx(ctx, ANN_CMD, 8, vec![format!("Data: {}", cmd[1]), format!("{}", cmd[1])])?;
        Ok(())
    }

    fn handle_software_reset_cmd(&mut self, ctx: &mut dyn Context) -> Result<()> {
        self.putx(ctx, ANN_CMD, 16, vec!["Software reset command".to_owned()])
    }

    fn handle_wake_up_timer_cmd(&mut self, ctx: &mut dyn Context, cmd: &[i64]) -> Result<()> {
        self.putx(ctx, ANN_CMD, 3, vec!["Wake-up timer command".to_owned(), "Timer".to_owned()])?;
        let r = cmd[0] & 0x1F;
        let m = cmd[1];
        let time = 1.03 * m as f64 * 2f64.powi(r as i32) + 0.5;
        self.putx(ctx, ANN_CMD, 13, vec![format!("Time: {time:7.2}")])?;
        Ok(())
    }

    fn handle_low_duty_cycle_cmd(&mut self, ctx: &mut dyn Context) -> Result<()> {
        self.putx(ctx, ANN_CMD, 16, vec!["Low duty cycle command".to_owned()])
    }

    fn handle_low_battery_detector_cmd(
        &mut self,
        ctx: &mut dyn Context,
        cmd: &[i64],
    ) -> Result<()> {
        self.putx(ctx, ANN_CMD, 8, vec!["Low battery detector command".to_owned()])?;
        const NAMES: [&str; 8] = ["1", "1.25", "1.66", "2", "2.5", "3.33", "5", "10"];
        let clock = NAMES[((cmd[1] & 0xE0) >> 5) as usize];
        self.putx(
            ctx,
            ANN_CMD,
            3,
            vec![format!("Clock output: {clock}MHz"), format!("{clock}MHz")],
        )?;
        self.advance_ann(ANN_CMD, 1);
        let v = 2.25 + (cmd[1] & 0x0F) as f64 * 0.1;
        self.putx(
            ctx,
            ANN_CMD,
            4,
            vec![format!("Low battery voltage: {v:.2}V"), format!("{v:.2}V")],
        )?;
        Ok(())
    }

    fn handle_status_read_cmd(&mut self, ctx: &mut dyn Context, ret: &[i64]) -> Result<()> {
        self.putx(ctx, ANN_CMD, 8, vec!["Status read command".to_owned(), "Status".to_owned()])?;
        const NAMES: [&str; 11] = [
            "RGIT/FFIT",
            "POR",
            "RGUR/FFOV",
            "WKUP",
            "EXT",
            "LBD",
            "FFEM",
            "RSSI/ATS",
            "DQD",
            "CRL",
            "ATGL",
        ];
        let status = (ret[0] << 3) + (ret[1] >> 5);
        self.row_pos[1] -= 8;
        self.row_pos[2] -= 8;
        self.describe_return_bits(ctx, status, &NAMES)?;
        let receiver_enabled = (self.last_power & 0x80) >> 7 != 0;

        for (mask, on, off) in [
            (0x80, None, None),
            (0x40, Some("Power on Reset"), Some("Power on Reset")),
            (0x20, None, None),
            (0x10, Some("Wake-up timer"), Some("Wake-up timer")),
            (0x08, Some("External interrupt"), Some("External interrupt")),
            (0x04, Some("Low battery"), Some("Low battery")),
            (0x02, Some("FIFO is empty"), Some("FIFO is empty")),
            (0x01, None, None),
        ] {
            if ret[0] & mask != 0 {
                let text = match (on, off) {
                    (Some(text), _) => text,
                    (None, _) => match (mask, receiver_enabled) {
                        (0x80, true) => "Received data in FIFO",
                        (0x80, false) => "Transmit register ready",
                        (0x20, true) => "RX FIFO overflow",
                        (0x20, false) => "Transmit register under run",
                        (0x01, true) => "Incoming signal above limit",
                        (0x01, false) => "Antenna detected RF signal",
                        _ => "",
                    },
                };
                self.putx(ctx, ANN_INTERPRETATION, 1, vec![text.to_owned()])?;
            } else {
                self.advance_ann(ANN_INTERPRETATION, 1);
            }
        }
        if ret[1] & 0x80 != 0 {
            self.putx(ctx, ANN_INTERPRETATION, 1, vec!["Data quality detector".to_owned()])?;
        } else {
            self.advance_ann(ANN_INTERPRETATION, 1);
        }
        if ret[1] & 0x40 != 0 {
            self.putx(ctx, ANN_INTERPRETATION, 1, vec!["Clock recovery locked".to_owned()])?;
        } else {
            self.advance_ann(ANN_INTERPRETATION, 1);
        }
        self.advance_ann(ANN_INTERPRETATION, 1);

        self.putx(ctx, ANN_RETURN, 5, vec!["AFC offset".to_owned()])?;
        if (self.last_status[1] & 0x1F) != (ret[1] & 0x1F) {
            self.putx(ctx, ANN_INTERPRETATION, 5, vec!["Changed".to_owned(), "~".to_owned()])?;
        }
        self.last_status = [ret[0], ret[1]];
        Ok(())
    }

    fn handle_cmd(&mut self, ctx: &mut dyn Context, cmd: &[i64], ret: &[i64]) -> Result<()> {
        if cmd[0] == 0x80 {
            self.handle_configuration_cmd(ctx, cmd)
        } else if cmd[0] == 0x82 {
            self.handle_power_management_cmd(ctx, cmd)
        } else if cmd[0] & 0xF0 == 0xA0 {
            self.handle_frequency_setting_cmd(ctx, cmd)
        } else if cmd[0] == 0xC6 {
            self.handle_data_rate_cmd(ctx, cmd)
        } else if cmd[0] & 0xF8 == 0x90 {
            self.handle_receiver_control_cmd(ctx, cmd)
        } else if cmd[0] == 0xC2 {
            self.handle_data_filter_cmd(ctx, cmd)
        } else if cmd[0] == 0xCA {
            self.handle_fifo_and_reset_cmd(ctx, cmd)
        } else if cmd[0] == 0xCE {
            self.handle_synchron_pattern_cmd(ctx, cmd)
        } else if cmd[0] == 0xB0 {
            self.handle_fifo_read_cmd(ctx, cmd, ret)
        } else if cmd[0] == 0xC4 {
            self.handle_afc_cmd(ctx, cmd)
        } else if cmd[0] & 0xFE == 0x98 {
            self.handle_transceiver_control_cmd(ctx, cmd)
        } else if cmd[0] == 0xCC {
            self.handle_pll_setting_cmd(ctx, cmd)
        } else if cmd[0] == 0xB8 {
            self.handle_transmitter_register_cmd(ctx, cmd)
        } else if cmd[0] == 0xFE {
            self.handle_software_reset_cmd(ctx)
        } else if cmd[0] & 0xE0 == 0xE0 {
            self.handle_wake_up_timer_cmd(ctx, cmd)
        } else if cmd[0] == 0xC8 {
            self.handle_low_duty_cycle_cmd(ctx)
        } else if cmd[0] == 0xC0 {
            self.handle_low_battery_detector_cmd(ctx, cmd)
        } else if cmd[0] == 0x00 {
            self.handle_status_read_cmd(ctx, ret)
        } else {
            let c = format!("{:02x} {:02x}", cmd[0], cmd[1]);
            let r = format!("{:02x} {:02x}", ret[0], ret[1]);
            self.putx(ctx, ANN_CMD, 16, vec![format!("Unknown command: {c} (reply: {r})!")])
        }
    }
}

/// `ptype, mosi, miso = data`.
fn split_item(item: &Item) -> Result<(&str, &Item, &Item)> {
    let Item::List(parts) = item else {
        return Err(Error::arg("rfm12: item is not a list"));
    };
    let [ptype, mosi, miso] = parts.as_slice() else {
        return Err(Error::arg("rfm12: item is not a three element packet"));
    };
    let Item::Str(ptype) = ptype else {
        return Err(Error::arg("rfm12: packet type is not a string"));
    };
    Ok((ptype.as_str(), mosi, miso))
}

fn int(item: &Item) -> Result<i64> {
    match item {
        Item::Int(v) => Ok(*v),
        _ => Err(Error::arg(format!("rfm12: not a word: {item}"))),
    }
}

/// A `BITS` packet's bit list to `(value, ss, es)` triples.
fn bit_list(item: &Item) -> Result<Vec<Bit>> {
    let Item::List(bits) = item else {
        return Err(Error::arg("rfm12: bit packet is not a list"));
    };
    let mut out = Vec::with_capacity(bits.len());
    for bit in bits.iter().rev() {
        let Item::List(fields) = bit else {
            return Err(Error::arg("rfm12: bit is not a list"));
        };
        let [v, ss, es] = fields.as_slice() else {
            return Err(Error::arg("rfm12: bit is not a three element list"));
        };
        out.push((int(v)?, int(ss)?, int(es)?));
    }
    Ok(out)
}

impl Decoder for Rfm12 {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, _config: &Config) -> Result<()> {
        *self = Rfm12::default();
        Ok(())
    }

    fn decode(&mut self, _ctx: &mut dyn Context) -> Result<()> {
        Err(Error::not_supported("rfm12 is a stacked decoder on 'spi'"))
    }

    fn decode_item(
        &mut self,
        _ss: u64,
        _es: u64,
        item: &Item,
        ctx: &mut dyn Context,
    ) -> Result<()> {
        if self.dead {
            return Ok(());
        }

        let (ptype, mosi, miso) = split_item(item)?;

        // For now, only use DATA and BITS packets.
        if ptype != "DATA" && ptype != "BITS" {
            return Ok(());
        }

        // Store the individual bit values and ss/es numbers. The next
        // packet is guaranteed to be a 'DATA' packet belonging to this
        // 'BITS' one.
        if ptype == "BITS" {
            if !matches!(mosi, Item::None) {
                self.mosi_bits.extend(bit_list(mosi)?);
            }
            // pd.py also keeps miso_bits, but no handler reads them.
            return Ok(());
        }

        // Append new bytes.
        self.mosi_bytes.push(int(mosi)?);
        self.miso_bytes.push(int(miso)?);

        // All commands consist of 2 bytes.
        if self.mosi_bytes.len() < 2 {
            return Ok(());
        }

        self.row_pos = [0, 8, 8];

        let cmd = self.mosi_bytes.clone();
        let ret = self.miso_bytes.clone();
        if let Err(e) = self.handle_cmd(ctx, &cmd, &ret) {
            // pd.py would have ended with this exception; nothing after
            // it is decoded.
            self.dead = true;
            return Err(e);
        }

        self.mosi_bytes.clear();
        self.miso_bytes.clear();
        self.mosi_bits.clear();
        Ok(())
    }
}
