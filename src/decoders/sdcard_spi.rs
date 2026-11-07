// Counterpart in sigrok:
//   libsigrokdecode/decoders/sdcard_spi/pd.py,
//   libsigrokdecode/decoders/common/sdcard/mod.py,
//   Copyright (C) 2012-2014 Uwe Hermann <uwe@hermann-uwe.de>,
//   GPL-2.0-or-later.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the SD card SPI-mode command set and the decoder's
//   documented classes and texts.
#![allow(clippy::doc_markdown, clippy::struct_excessive_bools, clippy::unnecessary_wraps)]

//! SD card (SPI mode): the low-level SPI-mode command and data protocol.

use crate::core::{Error, Result};
use crate::decode::{AnnClass, AnnRow, Config, Context, Decoder, DecoderDef, DecoderInfo, Item};

const ANN_BIT_WARNINGS: usize = 134;
/// pd.py emits its bit warnings with class 135 although its annotation
/// tuple ends at 134. Emitting the same out-of-range class keeps the
/// outputs identical.
const ANN_OUT_OF_RANGE_WARNING: usize = 135;

/// `cmd_names` from common/sdcard/mod.py. `None` is the module's
/// missing-key default, rendered as `Unknown` by `cmd_name`.
const CMD_NAMES: [Option<&str>; 64] = [
    Some("GO_IDLE_STATE"),
    Some("SEND_OP_COND"),
    Some("ALL_SEND_CID"),
    Some("SEND_RELATIVE_ADDR"),
    Some("SET_DSR"),
    Some("IO_SEND_OP_COND"),
    Some("SWITCH_FUNC"),
    Some("SELECT/DESELECT_CARD"),
    Some("SEND_IF_COND"),
    Some("SEND_CSD"),
    Some("SEND_CID"),
    Some("VOLTAGE_SWITCH"),
    Some("STOP_TRANSMISSION"),
    Some("SEND_STATUS"),
    None,
    Some("GO_INACTIVE_STATE"),
    Some("SET_BLOCKLEN"),
    Some("READ_SINGLE_BLOCK"),
    Some("READ_MULTIPLE_BLOCK"),
    Some("SEND_TUNING_BLOCK"),
    Some("SPEED_CLASS_CONTROL"),
    None,
    None,
    Some("SET_BLOCK_COUNT"),
    Some("WRITE_BLOCK"),
    Some("WRITE_MULTIPLE_BLOCK"),
    Some("Reserved for manufacturer"),
    Some("PROGRAM_CSD"),
    Some("SET_WRITE_PROT"),
    Some("CLR_WRITE_PROT"),
    Some("SEND_WRITE_PROT"),
    None,
    Some("ERASE_WR_BLK_START"),
    Some("ERASE_WR_BLK_END"),
    Some("Reserved for CMD6"),
    Some("Reserved for CMD6"),
    Some("Reserved for CMD6"),
    Some("Reserved for CMD6"),
    Some("ERASE"),
    None,
    Some("Reserved for security specification"),
    None,
    Some("LOCK_UNLOCK"),
    None,
    None,
    None,
    None,
    None,
    None,
    None,
    Some("Reserved for CMD6"),
    None,
    Some("IO_RW_DIRECT"),
    Some("IO_RW_EXTENDED"),
    Some("Unknown"),
    Some("APP_CMD"),
    Some("GEN_CMD"),
    Some("Reserved for CMD6"),
    Some("READ_OCR"),
    Some("CRC_ON_OFF"),
    Some("Reserved for manufacturer"),
    Some("Reserved for manufacturer"),
    Some("Reserved for manufacturer"),
    Some("Reserved for manufacturer"),
];

/// `acmd_names` from common/sdcard/mod.py.
const ACMD_NAMES: [Option<&str>; 64] = [
    None,
    None,
    None,
    None,
    None,
    None,
    Some("SET_BUS_WIDTH"),
    None,
    None,
    None,
    None,
    None,
    None,
    Some("SD_STATUS"),
    Some("Reserved for Security Application"),
    Some("Reserved for Security Application"),
    Some("Reserved for Security Application"),
    None,
    Some("Reserved for SD security applications"),
    None,
    None,
    None,
    Some("SEND_NUM_WR_BLOCKS"),
    Some("SET_WR_BLK_ERASE_COUNT"),
    None,
    Some("Reserved for SD security applications"),
    Some("Reserved for SD security applications"),
    Some("Reserved for security specification"),
    Some("Reserved for security specification"),
    None,
    Some("Reserved for security specification"),
    Some("Reserved for security specification"),
    Some("Reserved for security specification"),
    Some("Reserved for security specification"),
    Some("Reserved for security specification"),
    Some("Reserved for security specification"),
    None,
    None,
    Some("Reserved for SD security applications"),
    None,
    None,
    Some("SD_SEND_OP_COND"),
    Some("SET_CLR_CARD_DETECT"),
    Some("Reserved for SD security applications"),
    Some("Reserved for SD security applications"),
    Some("Reserved for SD security applications"),
    Some("Reserved for SD security applications"),
    Some("Reserved for SD security applications"),
    Some("Reserved for SD security applications"),
    Some("Reserved for SD security applications"),
    Some("Unknown"),
    Some("SEND_SCR"),
    Some("Reserved for security specification"),
    Some("Reserved for security specification"),
    Some("Reserved for security specification"),
    Some("Non-existant"),
    Some("Reserved for security specification"),
    Some("Reserved for security specification"),
    Some("Reserved for security specification"),
    Some("Reserved for security specification"),
    Some("Unknown"),
    Some("Unknown"),
    Some("Unknown"),
    Some("Unknown"),
];

/// The decoder description. The annotation ids are generated as in
/// pd.py: `cmd0..cmd63`, `acmd0..acmd63`, then the reply classes.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "sdcard_spi",
    name: "SD card (SPI mode)",
    longname: "Secure Digital card (SPI mode)",
    desc: "Secure Digital card (SPI mode) low-level protocol.",
    license: "gplv2+",
    inputs: &["spi"],
    outputs: &[],
    tags: &["Memory"],
    channels: &[],
    optional_channels: &[],
    options: &[],
    annotations: ANNOTATIONS,
    annotation_rows: &[
        AnnRow { id: "bits", desc: "Bits", classes: &[133, 134] },
        AnnRow { id: "cmd-reply", desc: "Commands/replies", classes: CMD_REPLY_CLASSES },
    ],
    binary: &[],
    doc: "This decoder stacks on top of the 'spi' PD and decodes the SD card\n\
(SPI mode) low-level protocol.\n\
\n\
Most SD cards can be accessed via two different protocols/modes: SD mode\n\
or SPI mode.\n\
\n\
All SD cards are in SD mode upon powerup. They can be switched to SPI mode\n\
using a special method involving CMD0 (see spec). Once in SPI mode, the mode\n\
can no longer be changed without a power-cycle of the card.\n\
\n\
SPI mode properties (differences to SD mode):\n\
\x20* The 'sdcard_spi' PD stacks on top of the 'spi' PD. This is not possible\n\
\x20\x20\x20for the 'sdcard_sd' PD, as that protocol is not SPI related at all.\n\
\x20\x20\x20Hence 'sdcard_spi' and 'sdcard_sd' are two separate PDs.\n\
\x20* The state machines for SPI mode and SD mode are different.\n\
\x20* In SPI mode, data transfers are byte-oriented (commands/data are multiples\n\
\x20\x20\x20of 8 bits, with the CS# pin asserted respectively), unlike SD mode where\n\
\x20\x20\x20commands/data are bit-oriented with parallel transmission of 1 or 4 bits.\n\
\x20* While the SPI mode command set has some commands in common with the\n\
\x20\x20\x20SD mode command set, they are not the same and also not a subset/superset.\n\
\x20\x20\x20Some commands are only available in SD mode (e.g. CMD2), some only\n\
\x20\x20\x20in SPI mode (e.g. CMD1).\n\
\x20* Response types of commands also differ between SD mode and SPI mode.\n\
\x20\x20\x20E.g. CMD9 has an R2 response in SD mode, but R1 in SPI mode.\n\
\x20* The commands and functions in SD mode defined after version 2.00 of the\n\
\x20\x20\x20spec are NOT supported in SPI mode.\n\
\x20* SPI mode: The selected SD card ALWAYS responds to commands (unlike SD mode).\n\
\x20* Upon data retrieval problems (read operations) the card will respond with\n\
\x20\x20\x20an error response (and no data), as opposed to a timeout in SD mode.\n\
\x20* SPI mode: For every data block sent to the card (write operations) the host\n\
\x20\x20\x20gets a data response token from the card.\n\
\x20* SDSC: A data block can be max. one card write block, min. 1 byte.\n\
\x20* SDHC/SDXC: Block length is fixed to 512. The block length set by CMD16\n\
\x20\x20\x20is only used for CDM42 (not for memory data transfers). Thus, partial\n\
\x20\x20\x20read/write operations are disabled in SPI mode.\n\
\x20* SPI mode: Write protected commands (CMD28, CMD29, CMD30) are not supported.\n\
\x20* The SD mode state machine is NOT used. All commands that are supported\n\
\x20\x20\x20in SPI mode are always available.\n\
\x20* Per default the card is in CRC OFF mode. Exception: CMD0 (which is used to\n\
\x20\x20\x20switch to SPI mode) needs a valid CRC.\n\
\x20* The APP_CMD status bit is not available in SPI mode.\n\
\x20* TODO: Switch function command differences.\n\
\x20* In SPI mode cards cannot guarantee their speed class (the host should\n\
\x20\x20\x20assume class 0, no matter what the card indicates).\n\
\x20* The RCA register is not accessible in SPI mode.",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(SdcardSpi::default())
}

// The two generated tables cannot be built in a const context from
// strings without a macro, so spell them out as pd.py's comprehensions.
macro_rules! ann_classes {
    ($($id:literal => $desc:literal),* $(,)?) => {
        &[$(AnnClass { id: $id, desc: $desc }),*]
    };
}

static ANNOTATIONS: &[AnnClass] = ann_classes![
    "cmd0" => "CMD0", "cmd1" => "CMD1", "cmd2" => "CMD2", "cmd3" => "CMD3",
    "cmd4" => "CMD4", "cmd5" => "CMD5", "cmd6" => "CMD6", "cmd7" => "CMD7",
    "cmd8" => "CMD8", "cmd9" => "CMD9", "cmd10" => "CMD10", "cmd11" => "CMD11",
    "cmd12" => "CMD12", "cmd13" => "CMD13", "cmd14" => "CMD14", "cmd15" => "CMD15",
    "cmd16" => "CMD16", "cmd17" => "CMD17", "cmd18" => "CMD18", "cmd19" => "CMD19",
    "cmd20" => "CMD20", "cmd21" => "CMD21", "cmd22" => "CMD22", "cmd23" => "CMD23",
    "cmd24" => "CMD24", "cmd25" => "CMD25", "cmd26" => "CMD26", "cmd27" => "CMD27",
    "cmd28" => "CMD28", "cmd29" => "CMD29", "cmd30" => "CMD30", "cmd31" => "CMD31",
    "cmd32" => "CMD32", "cmd33" => "CMD33", "cmd34" => "CMD34", "cmd35" => "CMD35",
    "cmd36" => "CMD36", "cmd37" => "CMD37", "cmd38" => "CMD38", "cmd39" => "CMD39",
    "cmd40" => "CMD40", "cmd41" => "CMD41", "cmd42" => "CMD42", "cmd43" => "CMD43",
    "cmd44" => "CMD44", "cmd45" => "CMD45", "cmd46" => "CMD46", "cmd47" => "CMD47",
    "cmd48" => "CMD48", "cmd49" => "CMD49", "cmd50" => "CMD50", "cmd51" => "CMD51",
    "cmd52" => "CMD52", "cmd53" => "CMD53", "cmd54" => "CMD54", "cmd55" => "CMD55",
    "cmd56" => "CMD56", "cmd57" => "CMD57", "cmd58" => "CMD58", "cmd59" => "CMD59",
    "cmd60" => "CMD60", "cmd61" => "CMD61", "cmd62" => "CMD62", "cmd63" => "CMD63",
    "acmd0" => "ACMD0", "acmd1" => "ACMD1", "acmd2" => "ACMD2", "acmd3" => "ACMD3",
    "acmd4" => "ACMD4", "acmd5" => "ACMD5", "acmd6" => "ACMD6", "acmd7" => "ACMD7",
    "acmd8" => "ACMD8", "acmd9" => "ACMD9", "acmd10" => "ACMD10", "acmd11" => "ACMD11",
    "acmd12" => "ACMD12", "acmd13" => "ACMD13", "acmd14" => "ACMD14", "acmd15" => "ACMD15",
    "acmd16" => "ACMD16", "acmd17" => "ACMD17", "acmd18" => "ACMD18", "acmd19" => "ACMD19",
    "acmd20" => "ACMD20", "acmd21" => "ACMD21", "acmd22" => "ACMD22", "acmd23" => "ACMD23",
    "acmd24" => "ACMD24", "acmd25" => "ACMD25", "acmd26" => "ACMD26", "acmd27" => "ACMD27",
    "acmd28" => "ACMD28", "acmd29" => "ACMD29", "acmd30" => "ACMD30", "acmd31" => "ACMD31",
    "acmd32" => "ACMD32", "acmd33" => "ACMD33", "acmd34" => "ACMD34", "acmd35" => "ACMD35",
    "acmd36" => "ACMD36", "acmd37" => "ACMD37", "acmd38" => "ACMD38", "acmd39" => "ACMD39",
    "acmd40" => "ACMD40", "acmd41" => "ACMD41", "acmd42" => "ACMD42", "acmd43" => "ACMD43",
    "acmd44" => "ACMD44", "acmd45" => "ACMD45", "acmd46" => "ACMD46", "acmd47" => "ACMD47",
    "acmd48" => "ACMD48", "acmd49" => "ACMD49", "acmd50" => "ACMD50", "acmd51" => "ACMD51",
    "acmd52" => "ACMD52", "acmd53" => "ACMD53", "acmd54" => "ACMD54", "acmd55" => "ACMD55",
    "acmd56" => "ACMD56", "acmd57" => "ACMD57", "acmd58" => "ACMD58", "acmd59" => "ACMD59",
    "acmd60" => "ACMD60", "acmd61" => "ACMD61", "acmd62" => "ACMD62", "acmd63" => "ACMD63",
    "r1" => "R1 reply", "r1b" => "R1B reply", "r2" => "R2 reply", "r3" => "R3 reply",
    "r7" => "R7 reply", "bits" => "Bits", "bit-warnings" => "Bit warnings",
];

static CMD_REPLY_CLASSES: &[usize] = &[
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25,
    26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49,
    50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73,
    74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97,
    98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116,
    117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132,
];

/// One bit of a BITS packet: `[value, ss, es]`.
#[derive(Clone, Copy)]
struct Bit {
    value: u64,
    ss: u64,
    es: u64,
}

fn parse_bits(data: &Item) -> Result<Vec<Bit>> {
    let Item::List(items) = data else {
        return Err(Error::arg("sdcard_spi: BITS payload is not a list"));
    };
    let mut bits = Vec::with_capacity(items.len());
    for item in items {
        let Item::List(parts) = item else {
            return Err(Error::arg("sdcard_spi: BITS entry is not a list"));
        };
        let [Item::Int(value), Item::Int(ss), Item::Int(es)] = parts.as_slice() else {
            return Err(Error::arg("sdcard_spi: BITS entry is not [bit, ss, es]"));
        };
        bits.push(Bit { value: *value as u64, ss: *ss as u64, es: *es as u64 });
    }
    Ok(bits)
}

fn value(item: &Item) -> Result<u64> {
    match item {
        Item::Int(v) if *v >= 0 => Ok(*v as u64),
        Item::BigUint(bytes) => {
            let mut v = 0u64;
            for &b in bytes {
                v = v << 8 | u64::from(b);
            }
            Ok(v)
        }
        _ => Err(Error::arg(format!("sdcard_spi: not an SPI word: {item}"))),
    }
}

/// Python's `str()` of a list of ints, as `%s` produces it.
fn py_list(values: &[u64]) -> String {
    let body = values.iter().map(u64::to_string).collect::<Vec<_>>().join(", ");
    format!("[{body}]")
}

/// The state machine's states, named as in pd.py.
#[derive(Clone, Copy, PartialEq, Eq, Default)]
enum State {
    #[default]
    Idle,
    GetCommandToken,
    HandleCmd(u64),
    GetResponseR1,
    HandleDataBlockCmd17,
    HandleDataBlockCmd24,
    DataResponse,
    WaitWhileCardBusy,
}

/// The SD card SPI-mode decoder.
#[derive(Default)]
pub struct SdcardSpi {
    state: State,
    ss: u64,
    es: u64,
    ss_bit: u64,
    es_bit: u64,
    ss_cmd: u64,
    es_cmd: u64,
    ss_busy: u64,
    es_busy: u64,
    cmd_token: Vec<u64>,
    cmd_token_bits: Vec<Vec<Bit>>,
    is_acmd: bool,
    blocklen: u64,
    read_buf: Vec<u64>,
    cmd_str: String,
    is_cmd24: bool,
    cmd24_start_token_found: bool,
    is_cmd17: bool,
    cmd17_start_token_found: bool,
    busy_first_byte: bool,
    miso_bits: Vec<Bit>,
    mosi_bits: Vec<Bit>,
    miso: u64,
    arg: u64,
    ss_data: u64,
    es_data: u64,
    ss_crc: u64,
    es_crc: u64,
}

impl SdcardSpi {
    fn putx(&self, ctx: &mut dyn Context, class: usize, texts: Vec<String>) {
        ctx.put_ann(self.ss_cmd, self.es_cmd, class, texts);
    }

    fn putc(&self, ctx: &mut dyn Context, cmd: usize, desc: &str) {
        ctx.put_ann(self.ss_cmd, self.es_cmd, cmd, vec![format!("{}: {}", self.cmd_str, desc)]);
    }

    fn putb(&self, ctx: &mut dyn Context, class: usize, text: String) {
        ctx.put_ann(self.ss_bit, self.es_bit, class, vec![text]);
    }

    fn cmd_name(&self, cmd: u64) -> String {
        let table = if self.is_acmd { &ACMD_NAMES } else { &CMD_NAMES };
        let name = table[cmd as usize].unwrap_or("Unknown");
        // SD mode names for CMD32/33 are ERASE_WR_BLK_{START,END}; SPI
        // mode names append _ADDR.
        if cmd == 32 || cmd == 33 { format!("{name}_ADDR") } else { name.to_owned() }
    }

    fn tb(&self, byte: usize, bit: usize) -> Bit {
        self.cmd_token_bits[5 - byte][bit]
    }

    fn handle_command_token(&mut self, mosi: u64, _miso: u64, ctx: &mut dyn Context) -> Result<()> {
        // Command tokens (6 bytes) are sent (MSB-first) by the host.
        if self.cmd_token.is_empty() {
            self.ss_cmd = self.ss;
        }

        self.cmd_token.push(mosi);
        self.cmd_token_bits.push(self.mosi_bits.clone());

        // All command tokens are 6 bytes long.
        if self.cmd_token.len() < 6 {
            return Ok(());
        }

        self.es_cmd = self.es;

        let t = self.cmd_token.clone();
        let s = if self.is_acmd { "ACMD" } else { "CMD" };

        // Bits[47:47]: Start bit (always 0)
        let b = self.tb(5, 7);
        let (bit, ss, es) = (b.value, b.ss, b.es);
        self.ss_bit = ss;
        self.es_bit = es;
        if bit == 0 {
            self.putb(ctx, ANN_BIT_WARNINGS, format!("Start bit: {bit}"));
        } else {
            self.putb(
                ctx,
                ANN_OUT_OF_RANGE_WARNING,
                format!("Start bit: {bit} (Warning: Must be 0!)"),
            );
        }

        // Bits[46:46]: Transmitter bit (1 == host)
        let b = self.tb(5, 6);
        let (bit, ss, es) = (b.value, b.ss, b.es);
        self.ss_bit = ss;
        self.es_bit = es;
        if bit == 1 {
            self.putb(ctx, ANN_BIT_WARNINGS, format!("Transmitter bit: {bit}"));
        } else {
            self.putb(
                ctx,
                ANN_OUT_OF_RANGE_WARNING,
                format!("Transmitter bit: {bit} (Warning: Must be 1!)"),
            );
        }

        // Bits[45:40]: Command index (BCD; valid: 0-63)
        let cmd = t[0] & 0x3f;
        self.ss_bit = self.tb(5, 5).ss;
        self.es_bit = self.tb(5, 0).es;
        self.putb(ctx, ANN_BIT_WARNINGS, format!("Command: {s}{cmd} ({})", self.cmd_name(cmd)));

        // Bits[39:8]: Argument
        self.arg = (t[1] << 24) | (t[2] << 16) | (t[3] << 8) | t[4];
        self.ss_bit = self.tb(4, 7).ss;
        self.es_bit = self.tb(1, 0).es;
        self.putb(ctx, ANN_BIT_WARNINGS, format!("Argument: 0x{:04x}", self.arg));

        // Bits[7:1]: CRC7. pd.py does not check CRC7.
        let crc = t[5] >> 1;
        self.ss_bit = self.tb(0, 7).ss;
        self.es_bit = self.tb(0, 1).es;
        self.putb(ctx, ANN_BIT_WARNINGS, format!("CRC7: 0x{crc:01x}"));

        // Bits[0:0]: End bit (always 1)
        let b = self.tb(0, 0);
        let (bit, ss, es) = (b.value, b.ss, b.es);
        self.ss_bit = ss;
        self.es_bit = es;
        if bit == 1 {
            self.putb(ctx, ANN_BIT_WARNINGS, format!("End bit: {bit}"));
        } else {
            self.putb(
                ctx,
                ANN_OUT_OF_RANGE_WARNING,
                format!("End bit: {bit} (Warning: Must be 1!)"),
            );
        }

        // Handle command.
        if matches!(cmd, 0 | 1 | 9 | 16 | 17 | 24 | 41 | 49 | 55 | 59) {
            self.state = State::HandleCmd(cmd);
            self.cmd_str = format!("{s}{cmd} ({})", self.cmd_name(cmd));
        } else {
            self.state = State::HandleCmd(999);
            let a = format!(
                "{s}{cmd}: {:02x} {:02x} {:02x} {:02x} {:02x} {:02x}",
                t[0], t[1], t[2], t[3], t[4], t[5]
            );
            self.putx(ctx, cmd as usize, vec![a]);
        }
        Ok(())
    }

    fn handle_cmd0(&mut self, ctx: &mut dyn Context) {
        self.putc(ctx, 0, "Reset the SD card");
        self.state = State::GetResponseR1;
    }

    fn handle_cmd1(&mut self, ctx: &mut dyn Context) {
        self.putc(ctx, 1, "Send HCS info and activate the card init process");
        let hcs = (self.arg & (1 << 30)) >> 30;
        self.ss_bit = self.cmd_token_bits[1][6].ss;
        self.es_bit = self.cmd_token_bits[1][6].es;
        self.putb(ctx, ANN_BIT_WARNINGS, format!("HCS: {hcs}"));
        self.state = State::GetResponseR1;
    }

    fn handle_cmd9(&mut self, ctx: &mut dyn Context) {
        self.putc(ctx, 9, "Ask card to send its card specific data (CSD)");
        if self.read_buf.is_empty() {
            self.ss_cmd = self.ss;
        }
        self.read_buf.push(self.miso);
        // pd.py's FIXME: 16 data bytes plus the four leading bytes.
        if self.read_buf.len() < 20 {
            return;
        }
        self.es_cmd = self.es;
        self.read_buf = self.read_buf[4..].to_vec();
        let text = format!("CSD: {}", py_list(&self.read_buf));
        self.putx(ctx, 9, vec![text]);
        self.read_buf.clear();
        self.state = State::Idle;
    }

    fn handle_cmd16(&mut self, ctx: &mut dyn Context) {
        self.blocklen = self.arg;
        self.putc(ctx, 16, &format!("Set the block length to {} bytes", self.blocklen));
        self.state = State::GetResponseR1;
    }

    fn handle_cmd17(&mut self, ctx: &mut dyn Context) {
        self.putc(ctx, 17, &format!("Read a block from address 0x{:04x}", self.arg));
        self.is_cmd17 = true;
        self.state = State::GetResponseR1;
    }

    fn handle_cmd24(&mut self, ctx: &mut dyn Context) {
        self.putc(ctx, 24, &format!("Write a block to address 0x{:04x}", self.arg));
        self.is_cmd24 = true;
        self.state = State::GetResponseR1;
    }

    fn handle_cmd49(&mut self) {
        self.state = State::GetResponseR1;
    }

    fn handle_cmd55(&mut self, ctx: &mut dyn Context) {
        self.putc(ctx, 55, "Next command is an application-specific command");
        self.is_acmd = true;
        self.state = State::GetResponseR1;
    }

    fn handle_cmd59(&mut self, ctx: &mut dyn Context) {
        let crc_on_off = self.arg & 1;
        let s = if crc_on_off == 1 { "on" } else { "off" };
        self.putc(ctx, 59, &format!("Turn the SD card CRC option {s}"));
        self.state = State::GetResponseR1;
    }

    fn handle_acmd41(&mut self, ctx: &mut dyn Context) {
        self.putc(ctx, 64 + 41, "Send HCS info and activate the card init process");
        self.state = State::GetResponseR1;
    }

    fn handle_cmd999(&mut self) {
        self.state = State::GetResponseR1;
    }

    /// pd.py dispatches `handle_{a}cmd{cmdstr}` by name; a missing
    /// method is an AttributeError that ends the decoder.
    fn handle_cmd(&mut self, cmd: u64, ctx: &mut dyn Context) -> Result<()> {
        match (self.is_acmd, cmd) {
            (false, 0) => self.handle_cmd0(ctx),
            (false, 1) => self.handle_cmd1(ctx),
            (false, 9) => self.handle_cmd9(ctx),
            (false, 16) => self.handle_cmd16(ctx),
            (false, 17) => self.handle_cmd17(ctx),
            (false, 24) => self.handle_cmd24(ctx),
            (false, 49) => self.handle_cmd49(),
            (false, 55) => self.handle_cmd55(ctx),
            (false, 59) => self.handle_cmd59(ctx),
            (true, 41) => self.handle_acmd41(ctx),
            (_, 999) => self.handle_cmd999(),
            _ => {
                return Err(Error::arg(format!(
                    "sdcard_spi: no handler for {}CMD{cmd}",
                    if self.is_acmd { "A" } else { "" }
                )));
            }
        }
        Ok(())
    }

    fn handle_response_r1(&mut self, res: u64, ctx: &mut dyn Context) {
        // The R1 response token format (1 byte).
        self.ss_cmd = self.miso_bits[7].ss;
        self.es_cmd = self.miso_bits[0].es;
        self.putx(ctx, 65, vec![format!("R1: 0x{res:02x}")]);

        // Bit 0: 'In idle state' bit
        let s = if res & (1 << 0) != 0 { "" } else { "not " };
        self.put_bit(0, format!("Card is {s}in idle state"), ctx);

        // Bit 1: 'Erase reset' bit
        let s = if res & (1 << 1) != 0 { "" } else { "not " };
        self.put_bit(1, format!("Erase sequence {s}cleared"), ctx);

        // Bit 2: 'Illegal command' bit
        let s = if res & (1 << 2) != 0 { "I" } else { "No i" };
        self.put_bit(2, format!("{s}llegal command detected"), ctx);

        // Bit 3: 'Communication CRC error' bit
        let s = if res & (1 << 3) != 0 { "failed" } else { "was successful" };
        self.put_bit(3, format!("CRC check of last command {s}"), ctx);

        // Bit 4: 'Erase sequence error' bit
        let s = if res & (1 << 4) != 0 { "E" } else { "No e" };
        self.put_bit(4, format!("{s}rror in the sequence of erase commands"), ctx);

        // Bit 5: 'Address error' bit. pd.py tests bit 4 here.
        let s = if res & (1 << 4) != 0 { "M" } else { "No m" };
        self.put_bit(5, format!("{s}isaligned address used in command"), ctx);

        // Bit 6: 'Parameter error' bit. pd.py tests bit 4 here too.
        let s = if res & (1 << 4) != 0 { "" } else { "not " };
        self.put_bit(6, format!("Command argument {s}outside allowed range"), ctx);

        // Bit 7: Always set to 0
        self.put_bit(7, "Bit 7 (always 0)".to_owned(), ctx);

        if self.is_cmd17 {
            self.state = State::HandleDataBlockCmd17;
        }
        if self.is_cmd24 {
            self.state = State::HandleDataBlockCmd24;
        }
    }

    fn put_bit(&mut self, bit: usize, text: String, ctx: &mut dyn Context) {
        let b = self.miso_bits[bit];
        self.ss_bit = b.ss;
        self.es_bit = b.es;
        self.putb(ctx, ANN_BIT_WARNINGS, text);
    }

    fn handle_data_cmd17(&mut self, miso: u64, ctx: &mut dyn Context) {
        if self.cmd17_start_token_found {
            if self.read_buf.is_empty() {
                self.ss_data = self.ss;
                if self.blocklen == 0 {
                    // Assume a fixed block size when inspection of the
                    // previous traffic did not provide the parameter.
                    self.blocklen = 512;
                }
            }
            self.read_buf.push(miso);
            // Wait until block transfer completed.
            if (self.read_buf.len() as u64) < self.blocklen {
                return;
            }
            if (self.read_buf.len() as u64) == self.blocklen {
                self.es_data = self.es;
                let text = format!("Block data: {}", py_list(&self.read_buf));
                ctx.put_ann(self.ss_data, self.es_data, 17, vec![text]);
            } else if (self.read_buf.len() as u64) == self.blocklen + 1 {
                self.ss_crc = self.ss;
            } else if (self.read_buf.len() as u64) == self.blocklen + 2 {
                self.es_crc = self.es;
                ctx.put_ann(self.ss_crc, self.es_crc, 17, vec!["CRC".to_owned()]);
                self.state = State::Idle;
            }
        } else if miso == 0xfe {
            ctx.put_ann(self.ss, self.es, 17, vec!["Start Block".to_owned()]);
            self.cmd17_start_token_found = true;
        }
    }

    fn handle_data_cmd24(&mut self, mosi: u64, ctx: &mut dyn Context) {
        if self.cmd24_start_token_found {
            if self.read_buf.is_empty() {
                self.ss_data = self.ss;
                if self.blocklen == 0 {
                    self.blocklen = 512;
                }
            }
            self.read_buf.push(mosi);
            if (self.read_buf.len() as u64) < self.blocklen {
                return;
            }
            self.es_data = self.es;
            let text = format!("Block data: {}", py_list(&self.read_buf));
            ctx.put_ann(self.ss_data, self.es_data, 24, vec![text]);
            self.read_buf.clear();
            self.state = State::DataResponse;
        } else if mosi == 0xfe {
            ctx.put_ann(self.ss, self.es, 24, vec!["Start Block".to_owned()]);
            self.cmd24_start_token_found = true;
        }
    }

    fn handle_data_response(&mut self, miso: u64, ctx: &mut dyn Context) {
        // Data Response token (1 byte).
        let miso = miso & 0x1f;
        if miso & 0x11 != 0x01 {
            // This is not the byte we are waiting for.
            return;
        }
        let m = self.miso_bits.clone();
        ctx.put_ann(m[7].ss, m[5].es, ANN_BIT_WARNINGS, vec!["Don't care".to_owned()]);
        ctx.put_ann(m[4].ss, m[4].es, ANN_BIT_WARNINGS, vec!["Always 0".to_owned()]);
        if miso == 0x05 {
            ctx.put_ann(m[3].ss, m[1].es, ANN_BIT_WARNINGS, vec!["Data accepted".to_owned()]);
        } else if miso == 0x0b {
            ctx.put_ann(
                m[3].ss,
                m[1].es,
                ANN_BIT_WARNINGS,
                vec!["Data rejected (CRC error)".to_owned()],
            );
        } else if miso == 0x0d {
            ctx.put_ann(
                m[3].ss,
                m[1].es,
                ANN_BIT_WARNINGS,
                vec!["Data rejected (write error)".to_owned()],
            );
        }
        ctx.put_ann(m[0].ss, m[0].es, ANN_BIT_WARNINGS, vec!["Always 1".to_owned()]);
        if self.is_cmd24 {
            ctx.put_ann(self.ss, self.es, 24, vec!["Data Response".to_owned()]);
        }
        if self.is_cmd24 {
            // We just send a block of data to be written to the card,
            // this takes some time.
            self.state = State::WaitWhileCardBusy;
            self.busy_first_byte = true;
        } else {
            self.state = State::Idle;
        }
    }

    fn wait_while_busy(&mut self, miso: u64, ctx: &mut dyn Context) {
        if miso != 0 {
            if self.is_cmd24 {
                ctx.put_ann(self.ss_busy, self.es_busy, 24, vec!["Card is busy".to_owned()]);
            }
            self.state = State::Idle;
            return;
        }
        if self.busy_first_byte {
            self.ss_busy = self.ss;
            self.busy_first_byte = false;
        } else {
            self.es_busy = self.es;
        }
    }
}

/// Splits an item from the SPI decoder into `(ptype, data1, data2)`.
fn split_item(item: &Item) -> Result<(&str, &Item, &Item)> {
    let Item::List(parts) = item else {
        return Err(Error::arg("sdcard_spi: item is not a list"));
    };
    let [ptype, data1, data2] = parts.as_slice() else {
        return Err(Error::arg("sdcard_spi: item is not a three element packet"));
    };
    let Item::Str(ptype) = ptype else {
        return Err(Error::arg("sdcard_spi: packet type is not a string"));
    };
    Ok((ptype.as_str(), data1, data2))
}

impl Decoder for SdcardSpi {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, config: &Config) -> Result<()> {
        let _ = config;
        *self = SdcardSpi::default();
        Ok(())
    }

    fn decode(&mut self, ctx: &mut dyn Context) -> Result<()> {
        let _ = ctx;
        Err(Error::not_supported("sdcard_spi is a stacked decoder on 'spi'"))
    }

    fn decode_item(&mut self, ss: u64, es: u64, item: &Item, ctx: &mut dyn Context) -> Result<()> {
        let (ptype, data1, data2) = split_item(item)?;

        // For now, only use DATA and BITS packets.
        if ptype != "DATA" && ptype != "BITS" {
            return Ok(());
        }

        // Store the individual bit values and ss/es numbers. The next
        // packet is guaranteed to be a 'DATA' packet of this 'BITS' one.
        if ptype == "BITS" {
            self.miso_bits = parse_bits(data2)?;
            self.mosi_bits = parse_bits(data1)?;
            return Ok(());
        }

        let mosi = value(data1)?;
        let miso = value(data2)?;
        self.ss = ss;
        self.es = es;

        // State machine.
        match self.state {
            State::Idle => {
                // Ignore stray 0xff bytes, some devices seem to send those!?
                if mosi == 0xff {
                    return Ok(());
                }
                self.state = State::GetCommandToken;
                self.handle_command_token(mosi, miso, ctx)
            }
            State::GetCommandToken => self.handle_command_token(mosi, miso, ctx),
            State::HandleCmd(cmd) => {
                self.miso = miso;
                let cmdstr = cmd;
                self.handle_cmd(cmd, ctx)?;
                self.cmd_token.clear();
                self.cmd_token_bits.clear();
                // Leave ACMD mode again after the first command after
                // CMD55.
                if self.is_acmd && cmdstr != 55 {
                    self.is_acmd = false;
                }
                Ok(())
            }
            State::GetResponseR1 => {
                // Ignore stray 0xff bytes, some devices seem to send those!?
                if miso == 0xff {
                    return Ok(());
                }
                self.state = State::Idle;
                self.handle_response_r1(miso, ctx);
                Ok(())
            }
            State::HandleDataBlockCmd17 => {
                self.handle_data_cmd17(miso, ctx);
                Ok(())
            }
            State::HandleDataBlockCmd24 => {
                self.handle_data_cmd24(mosi, ctx);
                Ok(())
            }
            State::DataResponse => {
                self.handle_data_response(miso, ctx);
                Ok(())
            }
            State::WaitWhileCardBusy => {
                self.wait_while_busy(miso, ctx);
                Ok(())
            }
        }
    }
}
