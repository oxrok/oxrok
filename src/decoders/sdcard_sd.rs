// Counterpart in sigrok:
//   libsigrokdecode/decoders/sdcard_sd/pd.py,
//   Copyright (C) 2015 Uwe Hermann <uwe@hermann-uwe.de>,
//   and decoders/common/sdcard/mod.py,
//   Copyright (C) 2012-2014 Uwe Hermann,
//   GPL-2.0-or-later.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the SD bus command/response formats and the
//   decoder's documented classes and texts.
#![allow(clippy::too_many_lines, clippy::unnecessary_wraps)]

//! SD card, SD mode: decodes the 48-bit command tokens and the
//! R1/R1b/R2/R3/R6/R7 responses on the CMD line.

use crate::core::Result;
use crate::decode::{
    AnnClass, AnnRow, ChannelDesc, Cond, CondKind, Config, Context, Decoder, DecoderDef,
    DecoderInfo,
};

const CMD: usize = 0;
const CLK: usize = 1;

// Annotation class indices, as the libsigrokdecode decoder numbers them.
const ANN_BITS: usize = 128;
const ANN_START: usize = 129;
const ANN_TRANSMISSION: usize = 130;
const ANN_CMD: usize = 131;
const ANN_ARG: usize = 132;
const ANN_CRC: usize = 133;
const ANN_END: usize = 134;
const ANN_DECODED_BIT: usize = 135;
const ANN_DECODED_FIELD: usize = 136;

const fn sequence_128() -> [usize; 128] {
    let mut out = [0usize; 128];
    let mut i = 0;
    while i < 128 {
        out[i] = i;
        i += 1;
    }
    out
}

static CMD_CLASSES: [usize; 128] = sequence_128();
static FIELD_CLASSES: [usize; 6] = [129, 130, 131, 132, 133, 134];

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "sdcard_sd",
    name: "SD card (SD mode)",
    longname: "Secure Digital card (SD mode)",
    desc: "Secure Digital card (SD mode) low-level protocol.",
    license: "gplv2+",
    inputs: &["logic"],
    outputs: &[],
    tags: &["Memory"],
    channels: &[
        ChannelDesc { id: "cmd", name: "CMD", desc: "Command" },
        ChannelDesc { id: "clk", name: "CLK", desc: "Clock" },
    ],
    optional_channels: &[
        ChannelDesc { id: "dat0", name: "DAT0", desc: "Data pin 0" },
        ChannelDesc { id: "dat1", name: "DAT1", desc: "Data pin 1" },
        ChannelDesc { id: "dat2", name: "DAT2", desc: "Data pin 2" },
        ChannelDesc { id: "dat3", name: "DAT3", desc: "Data pin 3" },
    ],
    options: &[],
    annotations: &ANNOTATIONS,
    annotation_rows: &[
        AnnRow { id: "raw-bits", desc: "Raw bits", classes: &[128] },
        AnnRow { id: "decoded-bits", desc: "Decoded bits", classes: &[135] },
        AnnRow { id: "decoded-fields", desc: "Decoded fields", classes: &[136] },
        AnnRow { id: "fields", desc: "Fields", classes: &FIELD_CLASSES },
        AnnRow { id: "cmd", desc: "Commands", classes: &CMD_CLASSES },
    ],
    binary: &[],
    doc: "SD card (SD mode) low-level protocol decoder.",
};

static ANNOTATIONS: [AnnClass; 137] = [
    AnnClass { id: "cmd0", desc: "CMD0" },
    AnnClass { id: "cmd1", desc: "CMD1" },
    AnnClass { id: "cmd2", desc: "CMD2" },
    AnnClass { id: "cmd3", desc: "CMD3" },
    AnnClass { id: "cmd4", desc: "CMD4" },
    AnnClass { id: "cmd5", desc: "CMD5" },
    AnnClass { id: "cmd6", desc: "CMD6" },
    AnnClass { id: "cmd7", desc: "CMD7" },
    AnnClass { id: "cmd8", desc: "CMD8" },
    AnnClass { id: "cmd9", desc: "CMD9" },
    AnnClass { id: "cmd10", desc: "CMD10" },
    AnnClass { id: "cmd11", desc: "CMD11" },
    AnnClass { id: "cmd12", desc: "CMD12" },
    AnnClass { id: "cmd13", desc: "CMD13" },
    AnnClass { id: "cmd14", desc: "CMD14" },
    AnnClass { id: "cmd15", desc: "CMD15" },
    AnnClass { id: "cmd16", desc: "CMD16" },
    AnnClass { id: "cmd17", desc: "CMD17" },
    AnnClass { id: "cmd18", desc: "CMD18" },
    AnnClass { id: "cmd19", desc: "CMD19" },
    AnnClass { id: "cmd20", desc: "CMD20" },
    AnnClass { id: "cmd21", desc: "CMD21" },
    AnnClass { id: "cmd22", desc: "CMD22" },
    AnnClass { id: "cmd23", desc: "CMD23" },
    AnnClass { id: "cmd24", desc: "CMD24" },
    AnnClass { id: "cmd25", desc: "CMD25" },
    AnnClass { id: "cmd26", desc: "CMD26" },
    AnnClass { id: "cmd27", desc: "CMD27" },
    AnnClass { id: "cmd28", desc: "CMD28" },
    AnnClass { id: "cmd29", desc: "CMD29" },
    AnnClass { id: "cmd30", desc: "CMD30" },
    AnnClass { id: "cmd31", desc: "CMD31" },
    AnnClass { id: "cmd32", desc: "CMD32" },
    AnnClass { id: "cmd33", desc: "CMD33" },
    AnnClass { id: "cmd34", desc: "CMD34" },
    AnnClass { id: "cmd35", desc: "CMD35" },
    AnnClass { id: "cmd36", desc: "CMD36" },
    AnnClass { id: "cmd37", desc: "CMD37" },
    AnnClass { id: "cmd38", desc: "CMD38" },
    AnnClass { id: "cmd39", desc: "CMD39" },
    AnnClass { id: "cmd40", desc: "CMD40" },
    AnnClass { id: "cmd41", desc: "CMD41" },
    AnnClass { id: "cmd42", desc: "CMD42" },
    AnnClass { id: "cmd43", desc: "CMD43" },
    AnnClass { id: "cmd44", desc: "CMD44" },
    AnnClass { id: "cmd45", desc: "CMD45" },
    AnnClass { id: "cmd46", desc: "CMD46" },
    AnnClass { id: "cmd47", desc: "CMD47" },
    AnnClass { id: "cmd48", desc: "CMD48" },
    AnnClass { id: "cmd49", desc: "CMD49" },
    AnnClass { id: "cmd50", desc: "CMD50" },
    AnnClass { id: "cmd51", desc: "CMD51" },
    AnnClass { id: "cmd52", desc: "CMD52" },
    AnnClass { id: "cmd53", desc: "CMD53" },
    AnnClass { id: "cmd54", desc: "CMD54" },
    AnnClass { id: "cmd55", desc: "CMD55" },
    AnnClass { id: "cmd56", desc: "CMD56" },
    AnnClass { id: "cmd57", desc: "CMD57" },
    AnnClass { id: "cmd58", desc: "CMD58" },
    AnnClass { id: "cmd59", desc: "CMD59" },
    AnnClass { id: "cmd60", desc: "CMD60" },
    AnnClass { id: "cmd61", desc: "CMD61" },
    AnnClass { id: "cmd62", desc: "CMD62" },
    AnnClass { id: "cmd63", desc: "CMD63" },
    AnnClass { id: "acmd0", desc: "ACMD0" },
    AnnClass { id: "acmd1", desc: "ACMD1" },
    AnnClass { id: "acmd2", desc: "ACMD2" },
    AnnClass { id: "acmd3", desc: "ACMD3" },
    AnnClass { id: "acmd4", desc: "ACMD4" },
    AnnClass { id: "acmd5", desc: "ACMD5" },
    AnnClass { id: "acmd6", desc: "ACMD6" },
    AnnClass { id: "acmd7", desc: "ACMD7" },
    AnnClass { id: "acmd8", desc: "ACMD8" },
    AnnClass { id: "acmd9", desc: "ACMD9" },
    AnnClass { id: "acmd10", desc: "ACMD10" },
    AnnClass { id: "acmd11", desc: "ACMD11" },
    AnnClass { id: "acmd12", desc: "ACMD12" },
    AnnClass { id: "acmd13", desc: "ACMD13" },
    AnnClass { id: "acmd14", desc: "ACMD14" },
    AnnClass { id: "acmd15", desc: "ACMD15" },
    AnnClass { id: "acmd16", desc: "ACMD16" },
    AnnClass { id: "acmd17", desc: "ACMD17" },
    AnnClass { id: "acmd18", desc: "ACMD18" },
    AnnClass { id: "acmd19", desc: "ACMD19" },
    AnnClass { id: "acmd20", desc: "ACMD20" },
    AnnClass { id: "acmd21", desc: "ACMD21" },
    AnnClass { id: "acmd22", desc: "ACMD22" },
    AnnClass { id: "acmd23", desc: "ACMD23" },
    AnnClass { id: "acmd24", desc: "ACMD24" },
    AnnClass { id: "acmd25", desc: "ACMD25" },
    AnnClass { id: "acmd26", desc: "ACMD26" },
    AnnClass { id: "acmd27", desc: "ACMD27" },
    AnnClass { id: "acmd28", desc: "ACMD28" },
    AnnClass { id: "acmd29", desc: "ACMD29" },
    AnnClass { id: "acmd30", desc: "ACMD30" },
    AnnClass { id: "acmd31", desc: "ACMD31" },
    AnnClass { id: "acmd32", desc: "ACMD32" },
    AnnClass { id: "acmd33", desc: "ACMD33" },
    AnnClass { id: "acmd34", desc: "ACMD34" },
    AnnClass { id: "acmd35", desc: "ACMD35" },
    AnnClass { id: "acmd36", desc: "ACMD36" },
    AnnClass { id: "acmd37", desc: "ACMD37" },
    AnnClass { id: "acmd38", desc: "ACMD38" },
    AnnClass { id: "acmd39", desc: "ACMD39" },
    AnnClass { id: "acmd40", desc: "ACMD40" },
    AnnClass { id: "acmd41", desc: "ACMD41" },
    AnnClass { id: "acmd42", desc: "ACMD42" },
    AnnClass { id: "acmd43", desc: "ACMD43" },
    AnnClass { id: "acmd44", desc: "ACMD44" },
    AnnClass { id: "acmd45", desc: "ACMD45" },
    AnnClass { id: "acmd46", desc: "ACMD46" },
    AnnClass { id: "acmd47", desc: "ACMD47" },
    AnnClass { id: "acmd48", desc: "ACMD48" },
    AnnClass { id: "acmd49", desc: "ACMD49" },
    AnnClass { id: "acmd50", desc: "ACMD50" },
    AnnClass { id: "acmd51", desc: "ACMD51" },
    AnnClass { id: "acmd52", desc: "ACMD52" },
    AnnClass { id: "acmd53", desc: "ACMD53" },
    AnnClass { id: "acmd54", desc: "ACMD54" },
    AnnClass { id: "acmd55", desc: "ACMD55" },
    AnnClass { id: "acmd56", desc: "ACMD56" },
    AnnClass { id: "acmd57", desc: "ACMD57" },
    AnnClass { id: "acmd58", desc: "ACMD58" },
    AnnClass { id: "acmd59", desc: "ACMD59" },
    AnnClass { id: "acmd60", desc: "ACMD60" },
    AnnClass { id: "acmd61", desc: "ACMD61" },
    AnnClass { id: "acmd62", desc: "ACMD62" },
    AnnClass { id: "acmd63", desc: "ACMD63" },
    AnnClass { id: "bits", desc: "Bits" },
    AnnClass { id: "field-start", desc: "Start bit" },
    AnnClass { id: "field-transmission", desc: "Transmission bit" },
    AnnClass { id: "field-cmd", desc: "Command" },
    AnnClass { id: "field-arg", desc: "Argument" },
    AnnClass { id: "field-crc", desc: "CRC" },
    AnnClass { id: "field-end", desc: "End bit" },
    AnnClass { id: "decoded-bits", desc: "Decoded bits" },
    AnnClass { id: "decoded-fields", desc: "Decoded fields" },
];

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(SdcardSd::default())
}

/// One bit of a command or response token: its level, and the sample
/// range the decoder saw it in.
#[derive(Clone, Copy)]
struct Token {
    ss: u64,
    es: u64,
    bit: u8,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum State {
    GetCommandToken,
    HandleCommand(u8),
    GetResponse(Response),
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Response {
    R1,
    #[allow(dead_code)]
    R1b,
    R2,
    R3,
    R6,
    R7,
}

/// The SD card (SD mode) decoder.
pub struct SdcardSd {
    state: State,
    token: Vec<Token>,
    is_acmd: bool,
    cmd: Option<u8>,
    last_cmd: Option<usize>,
    arg: Option<u32>,
    cmd_str: String,
}

impl Default for SdcardSd {
    fn default() -> Self {
        SdcardSd {
            state: State::GetCommandToken,
            token: Vec::new(),
            is_acmd: false,
            cmd: None,
            last_cmd: None,
            arg: None,
            cmd_str: String::new(),
        }
    }
}

const CMD_NAMES: [&str; 64] = [
    "GO_IDLE_STATE",
    "SEND_OP_COND",
    "ALL_SEND_CID",
    "SEND_RELATIVE_ADDR",
    "SET_DSR",
    "IO_SEND_OP_COND",
    "SWITCH_FUNC",
    "SELECT/DESELECT_CARD",
    "SEND_IF_COND",
    "SEND_CSD",
    "SEND_CID",
    "VOLTAGE_SWITCH",
    "STOP_TRANSMISSION",
    "SEND_STATUS",
    "Unknown",
    "GO_INACTIVE_STATE",
    "SET_BLOCKLEN",
    "READ_SINGLE_BLOCK",
    "READ_MULTIPLE_BLOCK",
    "SEND_TUNING_BLOCK",
    "SPEED_CLASS_CONTROL",
    "Unknown",
    "Unknown",
    "SET_BLOCK_COUNT",
    "WRITE_BLOCK",
    "WRITE_MULTIPLE_BLOCK",
    "Reserved for manufacturer",
    "PROGRAM_CSD",
    "SET_WRITE_PROT",
    "CLR_WRITE_PROT",
    "SEND_WRITE_PROT",
    "Unknown",
    "ERASE_WR_BLK_START",
    "ERASE_WR_BLK_END",
    "Reserved for CMD6",
    "Reserved for CMD6",
    "Reserved for CMD6",
    "Reserved for CMD6",
    "ERASE",
    "Unknown",
    "Reserved for security specification",
    "Unknown",
    "LOCK_UNLOCK",
    "Unknown",
    "Unknown",
    "Unknown",
    "Unknown",
    "Unknown",
    "Unknown",
    "Unknown",
    "Reserved for CMD6",
    "Unknown",
    "IO_RW_DIRECT",
    "IO_RW_EXTENDED",
    "Unknown",
    "APP_CMD",
    "GEN_CMD",
    "Reserved for CMD6",
    "READ_OCR",
    "CRC_ON_OFF",
    "Reserved for manufacturer",
    "Reserved for manufacturer",
    "Reserved for manufacturer",
    "Reserved for manufacturer",
];

const ACMD_NAMES: [&str; 64] = [
    "Unknown",
    "Unknown",
    "Unknown",
    "Unknown",
    "Unknown",
    "Unknown",
    "SET_BUS_WIDTH",
    "Unknown",
    "Unknown",
    "Unknown",
    "Unknown",
    "Unknown",
    "Unknown",
    "SD_STATUS",
    "Reserved for Security Application",
    "Reserved for Security Application",
    "Reserved for Security Application",
    "Unknown",
    "Reserved for SD security applications",
    "Unknown",
    "Unknown",
    "Unknown",
    "SEND_NUM_WR_BLOCKS",
    "SET_WR_BLK_ERASE_COUNT",
    "Unknown",
    "Reserved for SD security applications",
    "Reserved for SD security applications",
    "Reserved for security specification",
    "Reserved for security specification",
    "Unknown",
    "Reserved for security specification",
    "Reserved for security specification",
    "Reserved for security specification",
    "Reserved for security specification",
    "Reserved for security specification",
    "Reserved for security specification",
    "Unknown",
    "Unknown",
    "Reserved for SD security applications",
    "Unknown",
    "Unknown",
    "SD_SEND_OP_COND",
    "SET_CLR_CARD_DETECT",
    "Reserved for SD security applications",
    "Reserved for SD security applications",
    "Reserved for SD security applications",
    "Reserved for SD security applications",
    "Reserved for SD security applications",
    "Reserved for SD security applications",
    "Reserved for SD security applications",
    "Unknown",
    "SEND_SCR",
    "Reserved for security specification",
    "Reserved for security specification",
    "Reserved for security specification",
    "Non-existant",
    "Reserved for security specification",
    "Reserved for security specification",
    "Reserved for security specification",
    "Reserved for security specification",
    "Unknown",
    "Unknown",
    "Unknown",
    "Unknown",
];

const CARD_STATUS: [&str; 32] = [
    "Reserved for manufacturer test mode",
    "Reserved for manufacturer test mode",
    "Reserved for application specific commands",
    "AKE_SEQ_ERROR",
    "Reserved for SDIO card",
    "APP_CMD",
    "Unknown",
    "Unknown",
    "READY_FOR_DATA",
    "CURRENT_STATE",
    "CURRENT_STATE",
    "CURRENT_STATE",
    "CURRENT_STATE",
    "ERASE_RESET",
    "CARD_ECC_DISABLED",
    "WP_ERASE_SKIP",
    "CSD_OVERWRITE",
    "Reserved for DEFERRED_RESPONSE",
    "Reserved",
    "ERROR",
    "CC_ERROR",
    "CARD_ECC_FAILED",
    "ILLEGAL_COMMAND",
    "COM_CRC_ERROR",
    "LOCK_UNLOCK_FAILED",
    "CARD_IS_LOCKED",
    "WP_VIOLATION",
    "ERASE_PARAM",
    "ERASE_SEQ_ERROR",
    "BLOCK_LEN_ERROR",
    "ADDRESS_ERROR",
    "OUT_OF_RANGE",
];

impl SdcardSd {
    fn cmd_name(&self, cmd: u8) -> &'static str {
        let table = if self.is_acmd { &ACMD_NAMES } else { &CMD_NAMES };
        table.get(cmd as usize).copied().unwrap_or("Unknown")
    }

    /// The bits `[from, to)` of the token as an integer, MSB first.
    fn bits(&self, from: usize, to: usize) -> u32 {
        let mut out = 0u32;
        for t in &self.token[from..to] {
            out = (out << 1) | u32::from(t.bit);
        }
        out
    }

    fn putf(&self, ctx: &mut dyn Context, s: usize, e: usize, class: usize, texts: Vec<String>) {
        ctx.put_ann(self.token[s].ss, self.token[e].es, class, texts);
    }

    fn putt(&self, ctx: &mut dyn Context, class: usize, texts: Vec<String>) {
        ctx.put_ann(self.token[0].ss, self.token[47].es, class, texts);
    }

    /// Maps a bit range within the 32-bit argument to the token bits,
    /// newest argument bit first.
    fn puta(&self, ctx: &mut dyn Context, s: usize, e: usize, class: usize, texts: Vec<String>) {
        ctx.put_ann(self.token[39 - e].ss, self.token[39 - s].es, class, texts);
    }

    fn putc(&mut self, ctx: &mut dyn Context, class: usize, desc: &str) {
        self.last_cmd = Some(class);
        let cmd_str = self.cmd_str.clone();
        let short = cmd_str.split(' ').next().unwrap_or("").to_owned();
        self.putt(ctx, class, vec![format!("{cmd_str}: {desc}"), cmd_str, short]);
    }

    fn putr(&self, ctx: &mut dyn Context, desc: &str) {
        if let Some(class) = self.last_cmd {
            self.putt(ctx, class, vec![format!("Reply: {desc}")]);
        }
    }

    fn putbit(&self, ctx: &mut dyn Context, bit: usize, text: &str) {
        ctx.put_ann(self.token[bit].ss, self.token[bit].es, ANN_DECODED_BIT, vec![text.to_owned()]);
    }

    /// The Python `get_token_bits`: append one bit and report whether
    /// the token is complete. The last bit's end is extended by the
    /// duration of the one before it.
    fn add_token_bit(&mut self, bit: u8, samplenum: u64, n: usize) -> bool {
        self.token.push(Token { ss: samplenum, es: samplenum, bit });
        let len = self.token.len();
        if len >= 2 {
            self.token[len - 2].es = samplenum;
        }
        if len < n {
            return false;
        }
        self.token[n - 1].es += self.token[n - 1].ss - self.token[n - 2].ss;
        true
    }

    fn handle_common_token_fields(&mut self, ctx: &mut dyn Context) {
        for bit in 0..self.token.len() {
            let text = self.token[bit].bit.to_string();
            self.putf(ctx, bit, bit, ANN_BITS, vec![text]);
        }

        // "CMD[47:47]: Start bit (always 0)".
        self.putf(
            ctx,
            0,
            0,
            ANN_START,
            vec!["Start bit".to_owned(), "Start".to_owned(), "S".to_owned()],
        );

        // "CMD[46:46]: Transmission bit (1 == host)".
        let t = if self.token[1].bit == 1 { "host" } else { "card" };
        self.putf(
            ctx,
            1,
            1,
            ANN_TRANSMISSION,
            vec![format!("Transmission: {t}"), format!("T: {t}"), "T".to_owned()],
        );

        // "CMD[45:40]: Command index (BCD; valid: 0-63)".
        let cmd = self.bits(2, 8) as u8;
        let c = format!("{} ({})", self.cmd_name(cmd), cmd);
        self.putf(
            ctx,
            2,
            7,
            ANN_CMD,
            vec![
                format!("Command: {c}"),
                format!("Cmd: {c}"),
                format!("CMD{cmd}"),
                "Cmd".to_owned(),
                "C".to_owned(),
            ],
        );
        self.cmd = Some(cmd);

        // "CMD[39:08]: Argument".
        let arg = self.bits(8, 40);
        self.putf(
            ctx,
            8,
            39,
            ANN_ARG,
            vec![format!("Argument: 0x{arg:08x}"), "Arg".to_owned(), "A".to_owned()],
        );
        self.arg = Some(arg);

        // "CMD[07:01]: CRC7".
        let crc = self.bits(40, 47) as u8;
        self.putf(
            ctx,
            40,
            46,
            ANN_CRC,
            vec![format!("CRC: 0x{crc:x}"), "CRC".to_owned(), "C".to_owned()],
        );

        // "CMD[00:00]: End bit (always 1)".
        self.putf(
            ctx,
            47,
            47,
            ANN_END,
            vec!["End bit".to_owned(), "End".to_owned(), "E".to_owned()],
        );
    }

    fn get_command_token(&mut self, ctx: &mut dyn Context, cmd: bool) -> Result<()> {
        if !self.add_token_bit(u8::from(cmd), ctx.samplenum(), 48) {
            return Ok(());
        }

        self.handle_common_token_fields(ctx);

        let s = if self.is_acmd { "ACMD" } else { "CMD" };
        let index = self.cmd.unwrap_or(0);
        self.cmd_str = format!("{s}{index} ({})", self.cmd_name(index));
        if [0u8, 2, 3, 6, 7, 8, 9, 10, 13, 41, 51, 55].contains(&index) {
            self.state = State::HandleCommand(index);
        } else {
            self.state = State::HandleCommand(0xff);
            let desc = format!("{s}{index}");
            self.putc(ctx, index as usize, &desc);
        }
        Ok(())
    }

    fn handle_command(&mut self, ctx: &mut dyn Context, index: u8) -> Result<()> {
        let acmd = self.is_acmd;
        match (acmd, index) {
            (false, 0) => {
                // CMD0 (GO_IDLE_STATE) -> no response.
                self.puta(ctx, 0, 31, ANN_DECODED_FIELD, stuff_bits());
                self.putc(ctx, 0, "Reset all SD cards");
                self.token.clear();
                self.state = State::GetCommandToken;
            }
            (false, 2) => {
                // CMD2 (ALL_SEND_CID) -> R2.
                self.puta(ctx, 0, 31, ANN_DECODED_FIELD, stuff_bits());
                self.putc(ctx, 2, "Ask card for CID number");
                self.token.clear();
                self.state = State::GetResponse(Response::R2);
            }
            (false, 3) => {
                // CMD3 (SEND_RELATIVE_ADDR) -> R6.
                self.puta(ctx, 0, 31, ANN_DECODED_FIELD, stuff_bits());
                self.putc(ctx, 3, "Ask card for new relative card address (RCA)");
                self.token.clear();
                self.state = State::GetResponse(Response::R6);
            }
            (false, 6) => {
                // CMD6 (SWITCH_FUNC) -> R1.
                self.putc(ctx, 6, "Switch/check card function");
                self.token.clear();
                self.state = State::GetResponse(Response::R1);
            }
            (false, 7) => {
                // CMD7 (SELECT/DESELECT_CARD) -> R1b.
                self.putc(ctx, 7, "Select / deselect card");
                self.token.clear();
                self.state = State::GetResponse(Response::R6);
            }
            (false, 8) => {
                // CMD8 (SEND_IF_COND) -> R7.
                self.puta(ctx, 12, 31, ANN_DECODED_FIELD, reserved());
                self.puta(
                    ctx,
                    8,
                    11,
                    ANN_DECODED_FIELD,
                    vec![
                        "Supply voltage".to_owned(),
                        "Voltage".to_owned(),
                        "VHS".to_owned(),
                        "V".to_owned(),
                    ],
                );
                self.puta(
                    ctx,
                    0,
                    7,
                    ANN_DECODED_FIELD,
                    vec![
                        "Check pattern".to_owned(),
                        "Check pat".to_owned(),
                        "Check".to_owned(),
                        "C".to_owned(),
                    ],
                );
                self.putc(ctx, 8, "Send interface condition to card");
                self.token.clear();
                self.state = State::GetResponse(Response::R7);
            }
            (false, 9) => {
                // CMD9 (SEND_CSD) -> R2.
                self.puta(ctx, 16, 31, ANN_DECODED_FIELD, vec!["RCA".to_owned(), "R".to_owned()]);
                self.puta(ctx, 0, 15, ANN_DECODED_FIELD, stuff_bits());
                self.putc(ctx, 9, "Send card-specific data (CSD)");
                self.token.clear();
                self.state = State::GetResponse(Response::R2);
            }
            (false, 10) => {
                // CMD10 (SEND_CID) -> R2. Upstream passes 9 to putc, which
                // also sets last_cmd to the CMD9 class; kept for parity.
                self.puta(ctx, 16, 31, ANN_DECODED_FIELD, vec!["RCA".to_owned(), "R".to_owned()]);
                self.puta(ctx, 0, 15, ANN_DECODED_FIELD, stuff_bits());
                self.putc(ctx, 9, "Send card identification data (CID)");
                self.token.clear();
                self.state = State::GetResponse(Response::R2);
            }
            (false, 13) => {
                // CMD13 (SEND_STATUS) -> R1.
                self.puta(ctx, 16, 31, ANN_DECODED_FIELD, vec!["RCA".to_owned(), "R".to_owned()]);
                self.puta(ctx, 0, 15, ANN_DECODED_FIELD, stuff_bits());
                self.putc(ctx, 13, "Send card status register");
                self.token.clear();
                self.state = State::GetResponse(Response::R1);
            }
            (false, 55) => {
                // CMD55 (APP_CMD) -> R1.
                self.puta(ctx, 16, 31, ANN_DECODED_FIELD, vec!["RCA".to_owned(), "R".to_owned()]);
                self.puta(ctx, 0, 15, ANN_DECODED_FIELD, stuff_bits());
                self.putc(ctx, 55, "Next command is an application-specific command");
                self.is_acmd = true;
                self.token.clear();
                self.state = State::GetResponse(Response::R1);
            }
            (true, 6) => {
                // ACMD6 (SET_BUS_WIDTH) -> R1.
                self.putc(ctx, 64 + 6, "Read SD config register (SCR)");
                self.token.clear();
                self.state = State::GetResponse(Response::R1);
            }
            (true, 13) => {
                // ACMD13 (SD_STATUS) -> R1.
                self.puta(ctx, 0, 31, ANN_DECODED_FIELD, stuff_bits());
                self.putc(ctx, 64 + 13, "Set SD status");
                self.token.clear();
                self.state = State::GetResponse(Response::R1);
            }
            (true, 41) => {
                // ACMD41 (SD_SEND_OP_COND) -> R3.
                self.puta(
                    ctx,
                    0,
                    23,
                    ANN_DECODED_FIELD,
                    vec![
                        "VDD voltage window".to_owned(),
                        "VDD volt".to_owned(),
                        "VDD".to_owned(),
                        "V".to_owned(),
                    ],
                );
                self.puta(ctx, 24, 24, ANN_DECODED_FIELD, vec!["S18R".to_owned()]);
                self.puta(ctx, 25, 27, ANN_DECODED_FIELD, reserved());
                self.puta(ctx, 28, 28, ANN_DECODED_FIELD, vec!["XPC".to_owned()]);
                self.puta(
                    ctx,
                    29,
                    29,
                    ANN_DECODED_FIELD,
                    vec![
                        "Reserved for eSD".to_owned(),
                        "Reserved".to_owned(),
                        "Res".to_owned(),
                        "R".to_owned(),
                    ],
                );
                self.puta(
                    ctx,
                    30,
                    30,
                    ANN_DECODED_FIELD,
                    vec![
                        "Host capacity support info".to_owned(),
                        "Host capacity".to_owned(),
                        "HCS".to_owned(),
                        "H".to_owned(),
                    ],
                );
                self.puta(ctx, 31, 31, ANN_DECODED_FIELD, reserved());
                self.putc(ctx, 64 + 41, "Send HCS info and activate the card init process");
                self.token.clear();
                self.state = State::GetResponse(Response::R3);
            }
            (true, 51) => {
                // ACMD51 (SEND_SCR) -> R1.
                self.putc(ctx, 64 + 51, "Read SD config register (SCR)");
                self.token.clear();
                self.state = State::GetResponse(Response::R1);
            }
            (_, 0xff) => {
                // `handle_cmd999` and `handle_acmd999`.
                self.token.clear();
                self.state = State::GetResponse(Response::R1);
            }
            _ => {
                // The Python decoder would raise AttributeError for a
                // state it has no handler for.
                return Err(crate::core::Error::arg(format!(
                    "no handler for {}CMD{index}",
                    if acmd { "A" } else { "" }
                )));
            }
        }
        if self.is_acmd && index != 55 && index != 63 {
            self.is_acmd = false;
        }
        Ok(())
    }

    fn handle_response(&mut self, ctx: &mut dyn Context, resp: Response, cmd: bool) -> Result<()> {
        match resp {
            Response::R1 => {
                if !self.add_token_bit(u8::from(cmd), ctx.samplenum(), 48) {
                    return Ok(());
                }
                self.handle_common_token_fields(ctx);
                self.putr(ctx, "R1");
                self.puta(
                    ctx,
                    0,
                    31,
                    ANN_DECODED_FIELD,
                    vec!["Card status".to_owned(), "Status".to_owned(), "S".to_owned()],
                );
                for i in 0..32 {
                    self.putbit(ctx, 8 + i, CARD_STATUS[31 - i]);
                }
                self.token.clear();
                self.state = State::GetCommandToken;
            }
            Response::R1b => {
                if !self.add_token_bit(u8::from(cmd), ctx.samplenum(), 48) {
                    return Ok(());
                }
                self.handle_common_token_fields(ctx);
                self.puta(
                    ctx,
                    0,
                    31,
                    ANN_DECODED_FIELD,
                    vec!["Card status".to_owned(), "Status".to_owned(), "S".to_owned()],
                );
                self.putr(ctx, "R1b");
                self.token.clear();
                self.state = State::GetCommandToken;
            }
            Response::R2 => {
                if !self.add_token_bit(u8::from(cmd), ctx.samplenum(), 136) {
                    return Ok(());
                }
                for bit in 0..self.token.len() {
                    let text = self.token[bit].bit.to_string();
                    self.putf(ctx, bit, bit, ANN_BITS, vec![text]);
                }
                self.putf(
                    ctx,
                    0,
                    0,
                    ANN_START,
                    vec!["Start bit".to_owned(), "Start".to_owned(), "S".to_owned()],
                );
                let t = if self.token[1].bit == 1 { "host" } else { "card" };
                self.putf(
                    ctx,
                    1,
                    1,
                    ANN_TRANSMISSION,
                    vec![format!("Transmission: {t}"), format!("T: {t}"), "T".to_owned()],
                );
                self.putf(ctx, 2, 7, ANN_CMD, reserved());
                self.putf(
                    ctx,
                    8,
                    134,
                    ANN_ARG,
                    vec!["Argument".to_owned(), "Arg".to_owned(), "A".to_owned()],
                );
                self.putf(
                    ctx,
                    135,
                    135,
                    ANN_END,
                    vec!["End bit".to_owned(), "End".to_owned(), "E".to_owned()],
                );
                self.putf(
                    ctx,
                    8,
                    134,
                    ANN_DECODED_FIELD,
                    vec!["CID/CSD register".to_owned(), "CID/CSD".to_owned(), "C".to_owned()],
                );
                // Upstream tags the whole R2 token with class 55.
                self.putf(ctx, 0, 135, 55, vec!["R2".to_owned()]);
                self.token.clear();
                self.state = State::GetCommandToken;
            }
            Response::R3 => {
                if !self.add_token_bit(u8::from(cmd), ctx.samplenum(), 48) {
                    return Ok(());
                }
                self.putr(ctx, "R3");
                for bit in 0..self.token.len() {
                    let text = self.token[bit].bit.to_string();
                    self.putf(ctx, bit, bit, ANN_BITS, vec![text]);
                }
                self.putf(
                    ctx,
                    0,
                    0,
                    ANN_START,
                    vec!["Start bit".to_owned(), "Start".to_owned(), "S".to_owned()],
                );
                let t = if self.token[1].bit == 1 { "host" } else { "card" };
                self.putf(
                    ctx,
                    1,
                    1,
                    ANN_TRANSMISSION,
                    vec![format!("Transmission: {t}"), format!("T: {t}"), "T".to_owned()],
                );
                self.putf(ctx, 2, 7, ANN_CMD, reserved());
                self.putf(
                    ctx,
                    8,
                    39,
                    ANN_ARG,
                    vec!["Argument".to_owned(), "Arg".to_owned(), "A".to_owned()],
                );
                self.putf(ctx, 40, 46, ANN_CRC, reserved());
                self.putf(
                    ctx,
                    47,
                    47,
                    ANN_END,
                    vec!["End bit".to_owned(), "End".to_owned(), "E".to_owned()],
                );
                self.puta(
                    ctx,
                    0,
                    31,
                    ANN_DECODED_FIELD,
                    vec![
                        "OCR register".to_owned(),
                        "OCR reg".to_owned(),
                        "OCR".to_owned(),
                        "O".to_owned(),
                    ],
                );
                self.token.clear();
                self.state = State::GetCommandToken;
            }
            Response::R6 => {
                if !self.add_token_bit(u8::from(cmd), ctx.samplenum(), 48) {
                    return Ok(());
                }
                self.handle_common_token_fields(ctx);
                self.puta(
                    ctx,
                    0,
                    15,
                    ANN_DECODED_FIELD,
                    vec!["Card status bits".to_owned(), "Status".to_owned(), "S".to_owned()],
                );
                self.puta(
                    ctx,
                    16,
                    31,
                    ANN_DECODED_FIELD,
                    vec!["Relative card address".to_owned(), "RCA".to_owned(), "R".to_owned()],
                );
                self.putr(ctx, "R6");
                self.token.clear();
                self.state = State::GetCommandToken;
            }
            Response::R7 => {
                if !self.add_token_bit(u8::from(cmd), ctx.samplenum(), 48) {
                    return Ok(());
                }
                self.handle_common_token_fields(ctx);

                self.putr(ctx, "R7");

                // "Arg[31:12]: Reserved bits (all-zero)."
                self.puta(ctx, 12, 31, ANN_DECODED_FIELD, reserved());

                // "Arg[11:08]: Voltage accepted."
                let v = self.bits(28, 32);
                let av = match v {
                    0b0001 => "2.7-3.6V",
                    0b0010 => "reserved for low voltage range",
                    0b0100 | 0b1000 => "reserved",
                    _ => "Unknown",
                };
                self.puta(
                    ctx,
                    8,
                    11,
                    ANN_DECODED_FIELD,
                    vec![
                        format!("Voltage accepted: {av}"),
                        "Voltage".to_owned(),
                        "Volt".to_owned(),
                        "V".to_owned(),
                    ],
                );

                // "Arg[07:00]: Echo-back of check pattern."
                self.puta(
                    ctx,
                    0,
                    7,
                    ANN_DECODED_FIELD,
                    vec![
                        "Echo-back of check pattern".to_owned(),
                        "Echo".to_owned(),
                        "E".to_owned(),
                    ],
                );

                self.token.clear();
                self.state = State::GetCommandToken;
            }
        }
        Ok(())
    }
}

fn stuff_bits() -> Vec<String> {
    vec!["Stuff bits".to_owned(), "Stuff".to_owned(), "SB".to_owned(), "S".to_owned()]
}

fn reserved() -> Vec<String> {
    vec!["Reserved".to_owned(), "Res".to_owned(), "R".to_owned()]
}

impl Decoder for SdcardSd {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, _config: &Config) -> Result<()> {
        Ok(())
    }

    fn decode(&mut self, ctx: &mut dyn Context) -> Result<()> {
        loop {
            // "Wait for a rising CLK edge."
            let pins = ctx.wait(&[&[Cond::pin(CLK, CondKind::Rising)]])?;
            let cmd = pins.get(CMD);

            match self.state {
                State::GetCommandToken => {
                    if self.token.is_empty() && cmd {
                        // "Wait for start bit (CMD = 0)."
                        continue;
                    }
                    self.get_command_token(ctx, cmd)?;
                }
                State::HandleCommand(index) => {
                    self.handle_command(ctx, index)?;
                }
                State::GetResponse(resp) => {
                    if self.token.is_empty() && cmd {
                        continue;
                    }
                    self.handle_response(ctx, resp, cmd)?;
                }
            }
        }
    }
}
