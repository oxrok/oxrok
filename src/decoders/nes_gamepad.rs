// Counterpart in sigrok:
//   libsigrokdecode/decoders/nes_gamepad/pd.py,
//   Copyright (C) 2019 Stephan Thiele <stephan.thiele@mailbox.org>,
//   GPL-2.0-or-later.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the NES gamepad shift protocol and the decoder's
//   documented classes and texts.

//! Nintendo Entertainment System gamepad button states clocked out on
//! an SPI bus.

use crate::core::{Error, Result};
use crate::decode::{
    AnnClass, AnnRow, Config, Context, Decoder, DecoderDef, DecoderInfo, Item, OptionDesc,
    OptionValue,
};

// Annotation class indices, as the libsigrokdecode decoder numbers them.
const ANN_BUTTON: usize = 0;
const ANN_NO_PRESS: usize = 1;
const ANN_NOT_CONNECTED: usize = 2;

const BUTTONS: [&str; 8] = ["A", "B", "Select", "Start", "North", "South", "West", "East"];

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "nes_gamepad",
    name: "NES gamepad",
    longname: "Nintendo Entertainment System gamepad",
    desc: "NES gamepad button states.",
    license: "gplv2+",
    inputs: &["spi"],
    outputs: &[],
    tags: &["Retro computing"],
    channels: &[],
    optional_channels: &[],
    options: &[OptionDesc {
        id: "variant",
        desc: "Gamepad variant",
        default: OptionValue::s("Standard gamepad"),
        values: &[OptionValue::s("Standard gamepad")],
    }],
    annotations: &[
        AnnClass { id: "button", desc: "Button states" },
        AnnClass { id: "no-press", desc: "No button press" },
        AnnClass { id: "not-connected", desc: "Gamepad unconnected" },
    ],
    annotation_rows: &[
        AnnRow { id: "buttons", desc: "Button states", classes: &[0] },
        AnnRow { id: "no-press", desc: "No button press", classes: &[1] },
        AnnRow { id: "not-connected", desc: "Gamepad unconnected", classes: &[2] },
    ],
    binary: &[],
    doc: concat!(
        "This decoder stacks on top of the 'spi' PD and decodes the button states\n",
        "of an NES gamepad.\n",
        "\n",
        "The SPI decoder needs to be configured as follows:\n",
        "\n",
        "Clock polarity = 1\n",
        "Clock phase    = 0\n",
        "Bit order      = msb-first\n",
        "Word size      = 8\n",
        "\n",
        "Chip-select is not used and must not be assigned to any channel.\n",
        "\n",
        "Hardware setup is as follows:\n",
        "        ___\n",
        "   GND |o     CUP |o o| VCC\n",
        " OUT 0 |o o| D3\n",
        "    D1 |o o| D4\n",
        "       -----\n",
        "NES Gamepad Connector\n",
        "\n",
        "VCC   - Power 5V\n",
        "GND   - Ground\n",
        "CUP   - Shift register clock (CLK)\n",
        "OUT 0 - Shift register latch (optional)\n",
        "D1    - Gamepad data (MOSI)\n",
        "D3    - Data (unused)\n",
        "D4    - Data (unused)\n",
        "\n",
        "Data pins D3 and D4 are not used by the standard gamepad but\n",
        "by special controllers like the Nintento Zapper light gun.",
    ),
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(NesGamepad::default())
}

/// The NES gamepad decoder.
#[derive(Default)]
pub struct NesGamepad {
    ss_block: u64,
    es_block: u64,
    /// Set when pd.py would have ended with an exception.
    dead: bool,
}

impl NesGamepad {
    fn putx(&self, ctx: &mut dyn Context, class: usize, text: &str) {
        ctx.put_ann(self.ss_block, self.es_block, class, vec![text.to_owned()]);
    }

    fn handle_data(&self, ctx: &mut dyn Context, value: i64) -> Result<()> {
        if value == 0xFF {
            self.putx(ctx, ANN_NO_PRESS, "No button is pressed");
            return Ok(());
        }

        if value == 0x00 {
            self.putx(ctx, ANN_NOT_CONNECTED, "Gamepad is not connected");
            return Ok(());
        }

        // pd.py formats the word as eight bits, most significant first,
        // and a zero bit means the button is pressed.
        let bits = format!("{value:08b}");
        let mut button_str = String::new();
        for (button_index, c) in bits.chars().enumerate() {
            let button_is_pressed = c == '0';
            if button_is_pressed {
                if !button_str.is_empty() {
                    button_str.push_str(" + ");
                }
                let name = BUTTONS.get(button_index).ok_or_else(|| {
                    Error::arg(format!("nes_gamepad: no button at bit {button_index}"))
                })?;
                button_str.push_str(name);
            }
        }

        self.putx(ctx, ANN_BUTTON, &button_str);
        Ok(())
    }
}

/// `ptype, mosi, miso = data`.
fn split_item(item: &Item) -> Result<(&str, &Item, &Item)> {
    let Item::List(parts) = item else {
        return Err(Error::arg("nes_gamepad: item is not a list"));
    };
    let [ptype, mosi, miso] = parts.as_slice() else {
        return Err(Error::arg("nes_gamepad: item is not a three element packet"));
    };
    let Item::Str(ptype) = ptype else {
        return Err(Error::arg("nes_gamepad: packet type is not a string"));
    };
    Ok((ptype.as_str(), mosi, miso))
}

impl Decoder for NesGamepad {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, _config: &Config) -> Result<()> {
        *self = NesGamepad::default();
        Ok(())
    }

    fn decode(&mut self, _ctx: &mut dyn Context) -> Result<()> {
        Err(Error::not_supported("nes_gamepad is a stacked decoder on 'spi'"))
    }

    fn decode_item(&mut self, ss: u64, es: u64, item: &Item, ctx: &mut dyn Context) -> Result<()> {
        if self.dead {
            return Ok(());
        }
        let (ptype, _mosi, miso) = split_item(item)?;

        // pd.py assigns these before checking the packet type.
        self.ss_block = ss;
        self.es_block = es;

        if ptype != "DATA" {
            return Ok(());
        }

        let miso = match miso {
            Item::Int(v) => *v,
            _ => return Err(Error::arg(format!("nes_gamepad: not a word: {miso}"))),
        };
        if let Err(e) = self.handle_data(ctx, miso) {
            self.dead = true;
            return Err(e);
        }
        Ok(())
    }
}
