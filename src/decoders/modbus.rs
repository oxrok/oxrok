// Counterpart in sigrok:
//   libsigrokdecode/decoders/modbus/pd.py,
//   Copyright (C) 2015 Bart de Waal <bart@waalamo.com>.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the Modbus RTU framing and the decoder's documented
//   classes and texts.

//! Modbus RTU: server/client frames, function codes, CRC.

use crate::core::{Error, Result};
use crate::decode::{
    AnnClass, AnnRow, Config, Context, Decoder, DecoderDef, DecoderInfo, Item, OptionDesc,
    OptionValue,
};

// The annotation classes are two blocks: server->client, then client->server.
const SC_BASE: usize = 0;
const CS_BASE: usize = 7;
const ANN_ERROR_INDICATION: usize = 14;

// Offsets inside one direction's block.
const OFF_SERVER: usize = 0;
const OFF_FUNCTION: usize = 1;
const OFF_CRC: usize = 2;
const OFF_ADDRESS: usize = 3;
const OFF_DATA: usize = 4;
const OFF_LENGTH: usize = 5;
const OFF_ERROR: usize = 6;

const RXTX_CHANNELS: [&str; 2] = ["RX", "TX"];

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "modbus",
    name: "Modbus",
    longname: "Modbus RTU over RS232/RS485",
    desc: "Modbus RTU protocol for industrial applications.",
    license: "gplv3+",
    inputs: &["uart"],
    outputs: &["modbus"],
    tags: &["Embedded/industrial"],
    channels: &[],
    optional_channels: &[],
    options: &[
        OptionDesc {
            id: "scchannel",
            desc: "Server -> client channel",
            default: OptionValue::s("RX"),
            values: &[OptionValue::s("RX"), OptionValue::s("TX")],
        },
        OptionDesc {
            id: "cschannel",
            desc: "Client -> server channel",
            default: OptionValue::s("TX"),
            values: &[OptionValue::s("RX"), OptionValue::s("TX")],
        },
        OptionDesc {
            id: "framegap",
            desc: "Inter-frame bit gap",
            default: OptionValue::i(28),
            values: &[],
        },
    ],
    annotations: &[
        AnnClass { id: "sc-server-id", desc: "" },
        AnnClass { id: "sc-function", desc: "" },
        AnnClass { id: "sc-crc", desc: "" },
        AnnClass { id: "sc-address", desc: "" },
        AnnClass { id: "sc-data", desc: "" },
        AnnClass { id: "sc-length", desc: "" },
        AnnClass { id: "sc-error", desc: "" },
        AnnClass { id: "cs-server-id", desc: "" },
        AnnClass { id: "cs-function", desc: "" },
        AnnClass { id: "cs-crc", desc: "" },
        AnnClass { id: "cs-address", desc: "" },
        AnnClass { id: "cs-data", desc: "" },
        AnnClass { id: "cs-length", desc: "" },
        AnnClass { id: "cs-error", desc: "" },
        AnnClass { id: "error-indication", desc: "" },
    ],
    annotation_rows: &[
        AnnRow { id: "sc", desc: "Server->client", classes: &[0, 1, 2, 3, 4, 5, 6] },
        AnnRow { id: "cs", desc: "Client->server", classes: &[7, 8, 9, 10, 11, 12, 13] },
        AnnRow { id: "error-indicator", desc: "Errors in frame", classes: &[14] },
    ],
    binary: &[],
    doc: "This decoder stacks on top of the 'uart' PD and decodes Modbus RTU,\n\
a protocol with a single a client and one or more servers.\n\
\n\
The RX channel will be checked for both client->server and server->client\n\
communication, the TX channel only for client->server.",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(Modbus::default())
}

/// A byte of the UART stream with its sample range.
#[derive(Clone, Copy)]
struct Byte {
    start: u64,
    end: u64,
    data: u8,
}

/// The parse control flow: `No_more_data` is the decoder's normal
/// "nothing more to annotate yet", `Hard` is an exception upstream does
/// not catch.
enum Flow {
    NoMoreData,
    Hard(String),
}

type PResult<T> = std::result::Result<T, Flow>;

fn hard<T>(message: &str) -> PResult<T> {
    Err(Flow::Hard(message.to_owned()))
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Direction {
    Sc,
    Cs,
}

/// An Application Data Unit: one message as it arrives byte by byte.
struct Adu {
    data: Vec<Byte>,
    last_read: u64,
    last_byte_put: isize,
    base: usize,
    minimum_length: usize,
    start_new_frame: bool,
    has_error: bool,
}

impl Adu {
    fn new(start: u64, base: usize) -> Self {
        Adu {
            data: Vec::new(),
            last_read: start,
            last_byte_put: -1,
            base,
            minimum_length: 4,
            start_new_frame: false,
            has_error: false,
        }
    }

    fn get(&self, index: isize) -> PResult<Byte> {
        let len = self.data.len() as isize;
        let index = if index < 0 { index + len } else { index };
        if index < 0 || index >= len {
            return hard("list index out of range");
        }
        Ok(self.data[index as usize])
    }

    fn byte(&self, index: isize) -> PResult<u8> {
        Ok(self.get(index)?.data)
    }

    /// The half word (16 bit) value starting `start` bytes in.
    fn half_word(&self, start: isize) -> PResult<u32> {
        if start + 1 > self.data.len() as isize - 1 {
            return Err(Flow::NoMoreData);
        }
        Ok(u32::from(self.byte(start)?) * 0x100 + u32::from(self.byte(start + 1)?))
    }

    /// `puti`: annotates from the byte after the last annotation to
    /// `byte_to_put`, then signals no more data.
    fn puti(
        &mut self,
        ctx: &mut dyn Context,
        byte_to_put: isize,
        off: usize,
        message: &str,
    ) -> PResult<()> {
        if byte_to_put > self.data.len() as isize - 1 {
            return Err(Flow::NoMoreData);
        }
        if off == OFF_ERROR {
            self.has_error = true;
        }
        if byte_to_put > self.last_byte_put {
            let start = self.get(self.last_byte_put + 1)?.start;
            let end = self.get(byte_to_put)?.end;
            ctx.put_ann(start, end, self.base + off, vec![message.to_owned()]);
            self.last_byte_put = byte_to_put;
            return Err(Flow::NoMoreData);
        }
        Ok(())
    }

    /// `putl`: annotates the last byte, formatted into `template`.
    fn putl(
        &mut self,
        ctx: &mut dyn Context,
        off: usize,
        template: &str,
        maximum: Option<isize>,
    ) -> PResult<()> {
        let last = self.data.len() as isize - 1;
        if let Some(maximum) = maximum
            && last > maximum
        {
            return Ok(());
        }
        let value = self.byte(-1)?;
        let message = format_last(template, value)?;
        self.puti(ctx, last, off, &message)
    }

    /// `check_crc`: `byte_to_put` is the second CRC byte.
    fn check_crc(&mut self, ctx: &mut dyn Context, byte_to_put: isize) -> PResult<()> {
        let (crc1, crc2) = self.calc_crc(byte_to_put)?;
        if self.byte(-2)? == crc1 && self.byte(-1)? == crc2 {
            self.puti(ctx, byte_to_put, OFF_CRC, "CRC correct")
        } else {
            self.puti(ctx, byte_to_put, OFF_ERROR, &format!("CRC should be {crc1} {crc2}"))
        }
    }

    /// The CRC as described in the spec; `last_byte` is the second CRC
    /// byte's index.
    fn calc_crc(&self, last_byte: isize) -> PResult<(u8, u8)> {
        if last_byte < 3 {
            // Every Modbus ADU is at least 4 long.
            return hard("Could not calculate CRC: message too short");
        }
        let mut result: u32 = 0xFFFF;
        let magic_number: u32 = 0xA001;
        let end = ((last_byte - 1) as usize).min(self.data.len());
        for byte in &self.data[..end] {
            result ^= u32::from(byte.data);
            for _ in 0..8 {
                let lsb = result & 1;
                result >>= 1;
                if lsb != 0 {
                    result ^= magic_number;
                }
            }
        }
        Ok((result as u8, ((result & 0xff00) >> 8) as u8))
    }

    /// `close`: called when the next message starts.
    fn close(&mut self, ctx: &mut dyn Context, opts: &Opts, message_overflow: u64) -> PResult<()> {
        if self.data.len() < self.minimum_length {
            if self.data.is_empty() {
                // Sometimes happens with noise, safe to ignore.
                return Ok(());
            }
            let start = self.get(self.last_byte_put)?.end;
            ctx.put_ann(
                start,
                message_overflow,
                self.base + OFF_ERROR,
                vec!["Message too short or not finished".to_owned()],
            );
            self.has_error = true;
        }
        if self.has_error && opts.scchannel != opts.cschannel {
            // Only mark frames when the two directions are separate.
            let start = self.data[0].start;
            let end = self.data[self.data.len() - 1].end;
            ctx.put_ann(start, end, ANN_ERROR_INDICATION, vec!["Frame contains error".to_owned()]);
        }
        if self.data.len() > 256 {
            let last = self.data.len() as isize - 1;
            // Upstream catches No_more_data here.
            let _ = self.puti(ctx, last, OFF_ERROR, "Modbus data frames are limited to 256 bytes");
        }
        Ok(())
    }

    fn add_data(
        &mut self,
        ctx: &mut dyn Context,
        ss: u64,
        es: u64,
        ptype: &str,
        item: &Item,
    ) -> Result<()> {
        self.last_read = es;
        if ptype == "DATA" {
            let value = data_value(item)?;
            self.data.push(Byte { start: ss, end: es, data: value });
            self.parse(ctx)?;
        }
        Ok(())
    }

    /// Runs the version's parser, treating `No_more_data` as done.
    fn parse(&mut self, ctx: &mut dyn Context) -> Result<()> {
        let result = if self.base == SC_BASE { self.parse_sc(ctx) } else { self.parse_cs(ctx) };
        match result {
            Ok(()) => Ok(()),
            Err(Flow::NoMoreData) => Ok(()),
            Err(Flow::Hard(message)) => Err(Error::arg(message)),
        }
    }

    /// Server -> client.
    fn parse_sc(&mut self, ctx: &mut dyn Context) -> PResult<()> {
        let server_id = self.byte(0)?;
        let message = if (1..=247).contains(&server_id) {
            format!("Slave ID: {server_id}")
        } else {
            // Upstream leaves the braces in this message.
            "Slave ID {} is invalid".to_owned()
        };
        self.puti(ctx, 0, OFF_SERVER, &message)?;

        let function = self.byte(1)?;
        match function {
            1 | 2 => self.parse_read_bits(ctx)?,
            3 | 4 | 23 => self.parse_read_registers(ctx)?,
            5 => self.parse_write_single_coil(ctx)?,
            6 => self.parse_write_single_register(ctx)?,
            7 => self.parse_read_exception_status(ctx)?,
            8 => self.parse_diagnostics(ctx)?,
            11 => self.parse_get_comm_event_counter(ctx)?,
            12 => self.parse_get_comm_event_log(ctx)?,
            15 | 16 => self.parse_write_multiple(ctx)?,
            17 => self.parse_report_server_id(ctx)?,
            22 => self.parse_mask_write_register(ctx)?,
            21 | 24 | 43 => self.parse_not_implemented(ctx)?,
            // Upstream: `elif function > 0x80`.
            f if f > 0x80 => self.parse_error(ctx)?,
            f => {
                self.puti(ctx, 1, OFF_ERROR, &format!("Unknown function: {f}"))?;
                self.putl(ctx, OFF_ERROR, "Unknown function", None)?;
            }
        }

        // If the message gets here without raising, it is too long.
        self.putl(ctx, OFF_ERROR, "Message too long", None)
    }

    fn parse_read_bits(&mut self, ctx: &mut dyn Context) -> PResult<()> {
        // The base class's minimum stays 4: upstream's `mimumum_length`
        // assignments are misspelled no-ops.
        let function = self.byte(1)?;
        if function == 1 {
            self.puti(ctx, 1, OFF_FUNCTION, "Function 1: Read Coils")?;
        } else {
            self.puti(ctx, 1, OFF_FUNCTION, "Function 2: Read Discrete Inputs")?;
        }

        let bytecount = self.byte(2)? as isize;
        self.minimum_length = (5 + bytecount) as usize; // 3 before data, 2 CRC.
        self.puti(ctx, 2, OFF_LENGTH, &format!("Byte count: {bytecount}"))?;

        self.putl(ctx, OFF_DATA, "{:08b}", Some(bytecount + 2))?;
        self.check_crc(ctx, bytecount + 4)
    }

    fn parse_read_registers(&mut self, ctx: &mut dyn Context) -> PResult<()> {
        let function = self.byte(1)?;
        if function == 3 {
            self.puti(ctx, 1, OFF_FUNCTION, "Function 3: Read Holding Registers")?;
        } else if function == 4 {
            self.puti(ctx, 1, OFF_FUNCTION, "Function 4: Read Input Registers")?;
        } else if function == 23 {
            self.puti(ctx, 1, OFF_FUNCTION, "Function 23: Read/Write Multiple Registers")?;
        }

        let bytecount = self.byte(2)? as isize;
        self.minimum_length = (5 + bytecount) as usize; // 3 before data, 2 CRC.
        if bytecount % 2 == 0 {
            self.puti(ctx, 2, OFF_LENGTH, &format!("Byte count: {bytecount}"))?;
        } else {
            self.puti(ctx, 2, OFF_ERROR, &format!("Error: Odd byte count ({bytecount})"))?;
        }

        // From here on out, we expect registers on 3 and 4, 5 and 6 etc.
        if self.data.len() % 2 == 1 {
            let register_value = self.half_word(-2)?;
            let message = format!("0x{register_value:04X} / {register_value}");
            self.putl(ctx, OFF_DATA, &message, Some(bytecount + 2))?;
        } else {
            return Err(Flow::NoMoreData);
        }

        self.check_crc(ctx, bytecount + 4)
    }

    fn parse_read_exception_status(&mut self, ctx: &mut dyn Context) -> PResult<()> {
        self.puti(ctx, 1, OFF_FUNCTION, "Function 7: Read Exception Status")?;
        let exception_status = self.byte(2)?;
        self.puti(ctx, 2, OFF_DATA, &format!("Exception status: {exception_status:08b}"))?;
        self.check_crc(ctx, 4)
    }

    fn parse_get_comm_event_counter(&mut self, ctx: &mut dyn Context) -> PResult<()> {
        self.puti(ctx, 1, OFF_FUNCTION, "Function 11: Get Comm Event Counter")?;

        let status = self.half_word(2)?;
        if status == 0x0000 {
            self.puti(ctx, 3, OFF_DATA, "Status: not busy")?;
        } else if status == 0xFFFF {
            self.puti(ctx, 3, OFF_DATA, "Status: busy")?;
        } else {
            self.puti(ctx, 3, OFF_ERROR, &format!("Bad status: 0x{status:04X}"))?;
        }

        let count = self.half_word(4)?;
        self.puti(ctx, 5, OFF_DATA, &format!("Event Count: {count}"))?;
        self.check_crc(ctx, 7)
    }

    fn parse_get_comm_event_log(&mut self, ctx: &mut dyn Context) -> PResult<()> {
        self.puti(ctx, 1, OFF_FUNCTION, "Function 12: Get Comm Event Log")?;

        let bytecount = self.byte(2)? as isize;
        self.puti(ctx, 2, OFF_LENGTH, &format!("Bytecount: {bytecount}"))?;
        // The typo'd mimumum assignment is a no-op upstream too.

        let status = self.half_word(3)?;
        if status == 0x0000 {
            self.puti(ctx, 4, OFF_DATA, "Status: not busy")?;
        } else if status == 0xFFFF {
            self.puti(ctx, 4, OFF_DATA, "Status: busy")?;
        } else {
            self.puti(ctx, 4, OFF_ERROR, &format!("Bad status: 0x{status:04X}"))?;
        }

        let event_count = self.half_word(5)?;
        self.puti(ctx, 6, OFF_DATA, &format!("Event Count: {event_count}"))?;

        let message_count = self.half_word(7)?;
        self.puti(ctx, 8, OFF_DATA, &format!("Message Count: {message_count}"))?;

        self.putl(ctx, OFF_DATA, &format!("Event: 0x{:02X}", self.byte(-1)?), Some(bytecount + 2))?;

        self.check_crc(ctx, bytecount + 4)
    }

    fn parse_write_multiple(&mut self, ctx: &mut dyn Context) -> PResult<()> {
        // Functions 15 and 16 are almost the same.
        let function = self.byte(1)?;
        let (data_unit, max_outputs, long_address_offset) =
            if function == 15 { ("Coils", 0x07B0, 10001) } else { ("Registers", 0x007B, 30001) };

        self.puti(
            ctx,
            1,
            OFF_FUNCTION,
            &format!("Function {function}: Write Multiple {data_unit}"),
        )?;

        let starting_address = self.half_word(2)?;
        let address_name = long_address_offset + starting_address;
        self.puti(
            ctx,
            3,
            OFF_ADDRESS,
            &format!("Start at address 0x{starting_address:X} / {address_name}"),
        )?;

        let quantity_of_outputs = self.half_word(4)?;
        if quantity_of_outputs <= max_outputs {
            self.puti(ctx, 5, OFF_DATA, &format!("Write {quantity_of_outputs} {data_unit}"))?;
        } else {
            self.puti(
                ctx,
                5,
                OFF_ERROR,
                &format!("Bad value: {quantity_of_outputs} {data_unit}. Max is {max_outputs}"),
            )?;
        }

        self.check_crc(ctx, 7)
    }

    fn parse_report_server_id(&mut self, ctx: &mut dyn Context) -> PResult<()> {
        self.puti(ctx, 1, OFF_FUNCTION, "Function 17: Report Server ID")?;

        let bytecount = self.byte(2)? as isize;
        self.puti(ctx, 2, OFF_LENGTH, &format!("Data is {bytecount} bytes long"))?;

        self.puti(ctx, 3, OFF_DATA, &format!("serverID: {}", self.byte(3)?))?;

        let run_indicator_status = self.byte(4)?;
        if run_indicator_status == 0x00 {
            self.puti(ctx, 4, OFF_DATA, "Run Indicator status: Off")?;
        } else if run_indicator_status == 0xFF {
            self.puti(ctx, 4, OFF_DATA, "Run Indicator status: On")?;
        } else {
            self.puti(
                ctx,
                4,
                OFF_ERROR,
                &format!("Bad Run Indicator status: 0x{run_indicator_status:X}"),
            )?;
        }

        let last = self.byte(-1)?;
        self.putl(
            ctx,
            OFF_DATA,
            &format!("Device specific data: {last}, \"{}\"", char::from(last)),
            Some(2 + bytecount),
        )?;

        self.check_crc(ctx, 4 + bytecount)
    }

    fn parse_error(&mut self, ctx: &mut dyn Context) -> PResult<()> {
        // The function code of an error is 0x80 above the call's.
        let functioncode = i64::from(self.byte(1)?) - 0x80;
        let functions = [
            (1, "Read Coils"),
            (2, "Read Discrete Inputs"),
            (3, "Read Holding Registers"),
            (4, "Read Input Registers"),
            (5, "Write Single Coil"),
            (6, "Write Single Register"),
            (7, "Read Exception Status"),
            (8, "Diagnostic"),
            (11, "Get Com Event Counter"),
            (12, "Get Com Event Log"),
            (15, "Write Multiple Coils"),
            (16, "Write Multiple Registers"),
            (17, "Report Slave ID"),
            (20, "Read File Record"),
            (21, "Write File Record"),
            (22, "Mask Write Register"),
            (23, "Read/Write Multiple Registers"),
            (24, "Read FIFO Queue"),
            (43, "Read Device Identification/Encapsulated Interface Transport"),
        ];
        let functionname = functions
            .iter()
            .find(|(c, _)| *c == functioncode)
            .map(|(_, n)| *n)
            .unwrap_or("Unknown function");
        self.puti(
            ctx,
            1,
            OFF_FUNCTION,
            &format!("Error for function {functioncode}: {functionname}"),
        )?;

        let error = i64::from(self.byte(2)?);
        let errorcodes = [
            (1, "Illegal Function"),
            (2, "Illegal Data Address"),
            (3, "Illegal Data Value"),
            (4, "Slave Device Failure"),
            (5, "Acknowledge"),
            (6, "Slave Device Busy"),
            (8, "Memory Parity Error"),
            (10, "Gateway Path Unavailable"),
            (11, "Gateway Target Device failed to respond"),
        ];
        let errorname =
            errorcodes.iter().find(|(c, _)| *c == error).map(|(_, n)| *n).unwrap_or("Unknown");
        self.puti(ctx, 2, OFF_DATA, &format!("Error {error}: {errorname}"))?;
        self.check_crc(ctx, 4)
    }

    /// Client -> server.
    fn parse_cs(&mut self, ctx: &mut dyn Context) -> PResult<()> {
        let server_id = self.byte(0)?;
        let message = if server_id == 0 {
            "Broadcast message".to_owned()
        } else if (1..=247).contains(&server_id) {
            format!("Slave ID: {server_id}")
        } else if (248..=255).contains(&server_id) {
            format!("Slave ID: {server_id} (reserved address)")
        } else {
            String::new()
        };
        self.puti(ctx, 0, OFF_SERVER, &message)?;

        let function = self.byte(1)?;
        if (1..=4).contains(&function) {
            self.parse_read_data_command(ctx)?;
        }
        if function == 5 {
            self.parse_write_single_coil(ctx)?;
        }
        if function == 6 {
            self.parse_write_single_register(ctx)?;
        }
        if matches!(function, 7 | 11 | 12 | 17) {
            self.parse_single_byte_request(ctx)?;
        } else if function == 8 {
            self.parse_diagnostics(ctx)?;
        }
        if matches!(function, 15 | 16) {
            self.parse_write_multiple_cs(ctx)?;
        } else if function == 22 {
            self.parse_mask_write_register(ctx)?;
        } else if function == 23 {
            self.parse_read_write_registers(ctx)?;
        } else if matches!(function, 21 | 24 | 43) {
            self.parse_not_implemented(ctx)?;
        } else {
            self.puti(ctx, 1, OFF_ERROR, &format!("Unknown function: {function}"))?;
            self.putl(ctx, OFF_ERROR, "Unknown function", None)?;
        }

        // If the message gets here without raising, it is too long.
        self.putl(ctx, OFF_ERROR, "Message too long", None)
    }

    fn parse_read_data_command(&mut self, ctx: &mut dyn Context) -> PResult<()> {
        self.minimum_length = 8;

        let function = self.byte(1)?;
        let functionname = match function {
            1 => "Read Coils",
            2 => "Read Discrete Inputs",
            3 => "Read Holding Registers",
            4 => "Read Input Registers",
            _ => return hard("KeyError"),
        };
        self.puti(ctx, 1, OFF_FUNCTION, &format!("Function {function}: {functionname}"))?;

        let starting_address = self.half_word(2)?;
        let address_name = 10000 * u32::from(function) + 1 + starting_address;
        self.puti(
            ctx,
            3,
            OFF_ADDRESS,
            &format!("Start at address 0x{starting_address:X} / {address_name}"),
        )?;

        self.puti(ctx, 5, OFF_LENGTH, &format!("Read {} units of data", self.half_word(4)?))?;
        self.check_crc(ctx, 7)
    }

    fn parse_single_byte_request(&mut self, ctx: &mut dyn Context) -> PResult<()> {
        let function = self.byte(1)?;
        let function_name = match function {
            7 => "Read Exception Status",
            11 => "Get Comm Event Counter",
            12 => "Get Comm Event Log",
            17 => "Report Slave ID",
            _ => return hard("KeyError"),
        };
        self.puti(ctx, 1, OFF_FUNCTION, &format!("Function {function}: {function_name}"))?;
        self.check_crc(ctx, 3)
    }

    fn parse_write_multiple_cs(&mut self, ctx: &mut dyn Context) -> PResult<()> {
        let function = self.byte(1)?;
        let (data_unit, max_outputs, ratio_bytes_data, long_address_offset) = if function == 15 {
            ("Coils", 0x07B0, 1.0 / 8.0, 10001)
        } else {
            ("Registers", 0x007B, 2.0, 30001)
        };

        self.puti(
            ctx,
            1,
            OFF_FUNCTION,
            &format!("Function {function}: Write Multiple {data_unit}"),
        )?;

        let starting_address = self.half_word(2)?;
        let address_name = long_address_offset + starting_address;
        self.puti(
            ctx,
            3,
            OFF_ADDRESS,
            &format!("Start at address 0x{starting_address:X} / {address_name}"),
        )?;

        let quantity_of_outputs = self.half_word(4)?;
        if quantity_of_outputs <= max_outputs {
            self.puti(ctx, 5, OFF_LENGTH, &format!("Write {quantity_of_outputs} {data_unit}"))?;
        } else {
            self.puti(
                ctx,
                5,
                OFF_ERROR,
                &format!("Bad value: {quantity_of_outputs} {data_unit}. Max is {max_outputs}"),
            )?;
        }
        let proper_bytecount = (f64::from(quantity_of_outputs) * ratio_bytes_data).ceil() as i64;

        let bytecount = self.byte(6)?;
        if i64::from(bytecount) == proper_bytecount {
            self.puti(ctx, 6, OFF_LENGTH, &format!("Byte count: {bytecount}"))?;
        } else {
            self.puti(
                ctx,
                6,
                OFF_ERROR,
                &format!("Bad byte count, is {bytecount}, should be {proper_bytecount}"),
            )?;
        }
        // The typo'd mimumum assignment is a no-op upstream too.

        self.putl(ctx, OFF_DATA, "Value 0x{:X}", Some(6 + isize::from(bytecount)))?;
        self.check_crc(ctx, isize::from(bytecount) + 8)
    }

    fn parse_read_write_registers(&mut self, ctx: &mut dyn Context) -> PResult<()> {
        self.minimum_length = 13;

        self.puti(ctx, 1, OFF_FUNCTION, "Function 23: Read/Write Multiple Registers")?;

        let starting_address = self.half_word(2)?;
        let address_name = 30001 + starting_address;
        self.puti(
            ctx,
            3,
            OFF_ADDRESS,
            &format!("Read starting at address 0x{starting_address:X} / {address_name}"),
        )?;

        self.puti(ctx, 5, OFF_LENGTH, &format!("Read {} units of data", self.half_word(4)?))?;

        let starting_address = self.half_word(6)?;
        // Upstream keeps using the read address's long name here.
        self.puti(
            ctx,
            7,
            OFF_ADDRESS,
            &format!("Write starting at address 0x{starting_address:X} / {address_name}"),
        )?;

        let quantity_of_outputs = self.half_word(8)?;
        self.puti(ctx, 9, OFF_LENGTH, &format!("Write {quantity_of_outputs} registers"))?;
        let proper_bytecount = i64::from(quantity_of_outputs) * 2;

        let bytecount = self.byte(10)?;
        if i64::from(bytecount) == proper_bytecount {
            self.puti(ctx, 10, OFF_LENGTH, &format!("Byte count: {bytecount}"))?;
        } else {
            self.puti(
                ctx,
                10,
                OFF_ERROR,
                &format!("Bad byte count, is {bytecount}, should be {proper_bytecount}"),
            )?;
        }
        // The typo'd mimumum assignment is a no-op upstream too.

        self.putl(ctx, OFF_DATA, "Data, value 0x{:02X}", Some(10 + isize::from(bytecount)))?;
        self.check_crc(ctx, isize::from(bytecount) + 12)
    }

    // Shared by both directions.
    fn parse_write_single_coil(&mut self, ctx: &mut dyn Context) -> PResult<()> {
        self.minimum_length = 8;

        self.puti(ctx, 1, OFF_FUNCTION, "Function 5: Write Single Coil")?;

        let address = self.half_word(2)?;
        self.puti(ctx, 3, OFF_ADDRESS, &format!("Address 0x{address:X} / {}", address + 10000))?;

        let raw_value = self.half_word(4)?;
        let value = match raw_value {
            0x0000 => "Coil Value OFF",
            0xFF00 => "Coil Value ON",
            _ => "Invalid Coil Value",
        };
        self.puti(ctx, 5, OFF_DATA, value)?;

        self.check_crc(ctx, 7)
    }

    fn parse_write_single_register(&mut self, ctx: &mut dyn Context) -> PResult<()> {
        self.minimum_length = 8;

        self.puti(ctx, 1, OFF_FUNCTION, "Function 6: Write Single Register")?;

        let address = self.half_word(2)?;
        self.puti(ctx, 3, OFF_ADDRESS, &format!("Address 0x{address:X} / {}", address + 30000))?;

        let value = self.half_word(4)?;
        self.puti(ctx, 5, OFF_DATA, &format!("Register Value 0x{value:X} / {value}"))?;

        self.check_crc(ctx, 7)
    }

    fn parse_diagnostics(&mut self, ctx: &mut dyn Context) -> PResult<()> {
        self.minimum_length = 8;

        self.puti(ctx, 1, OFF_FUNCTION, "Function 8: Diagnostics")?;

        let diag_subfunction = [
            (0, "Return Query data"),
            (1, "Restart Communications Option"),
            (2, "Return Diagnostics Register"),
            (3, "Change ASCII Input Delimiter"),
            (4, "Force Listen Only Mode"),
            (10, "Clear Counters and Diagnostic Register"),
            (11, "Return Bus Message Count"),
            (12, "Return Bus Communication Error Count"),
            (13, "Return Bus Exception Error Count"),
            (14, "Return Slave Message Count"),
            (15, "Return Slave No Response Count"),
            (16, "Return Slave NAK Count"),
            (17, "Return Slave Busy Count"),
            (18, "Return Bus Character Overrun Count"),
            (20, "Return Overrun Counter and Flag"),
        ];
        let subfunction = self.half_word(2)?;
        let subfunction_name = diag_subfunction
            .iter()
            .find(|(c, _)| *c == subfunction)
            .map(|(_, n)| *n)
            .unwrap_or("Reserved subfunction");
        self.puti(ctx, 3, OFF_DATA, &format!("Subfunction {subfunction}: {subfunction_name}"))?;

        let diagnostic_data = self.half_word(4)?;
        self.puti(
            ctx,
            5,
            OFF_DATA,
            &format!("Data Field: {diagnostic_data} / 0x{diagnostic_data:04X}"),
        )?;

        self.check_crc(ctx, 7)
    }

    fn parse_mask_write_register(&mut self, ctx: &mut dyn Context) -> PResult<()> {
        self.minimum_length = 10;

        self.puti(ctx, 1, OFF_FUNCTION, "Function 22: Mask Write Register")?;

        let address = self.half_word(2)?;
        self.puti(ctx, 3, OFF_ADDRESS, &format!("Address 0x{address:X} / {}", address + 30001))?;

        self.half_word(4)?; // To make sure we don't overflow data.
        let and_mask_1 = self.byte(4)?;
        let and_mask_2 = self.byte(5)?;
        self.puti(ctx, 5, OFF_DATA, &format!("AND mask: {and_mask_1:08b} {and_mask_2:08b}"))?;

        self.half_word(6)?; // To make sure we don't overflow data.
        let or_mask_1 = self.byte(6)?;
        let or_mask_2 = self.byte(7)?;
        self.puti(ctx, 7, OFF_DATA, &format!("OR mask: {or_mask_1:08b} {or_mask_2:08b}"))?;

        self.check_crc(ctx, 9)
    }

    fn parse_not_implemented(&mut self, ctx: &mut dyn Context) -> PResult<()> {
        let function = self.byte(1)?;
        let functionname = match function {
            20 => "Read File Record",
            21 => "Write File Record",
            24 => "Read FIFO Queue",
            43 => "Read Device Identification/Encapsulated Interface Transport",
            _ => return hard("KeyError"),
        };
        self.puti(
            ctx,
            1,
            OFF_FUNCTION,
            &format!("Function {function}: {functionname} (not supported)"),
        )?;
        self.putl(ctx, OFF_DATA, "This function is not currently supported", None)
    }
}

/// The parsed options.
#[derive(Clone)]
struct Opts {
    scchannel: String,
    cschannel: String,
    framegap: i64,
}

/// The Modbus decoder.
#[derive(Default)]
pub struct Modbus {
    opts: Option<Opts>,
    sc: Option<Adu>,
    cs: Option<Adu>,
    bitlength: Option<u64>,
}

impl Modbus {
    fn adu(&self, direction: Direction) -> &Adu {
        match direction {
            Direction::Sc => self.sc.as_ref(),
            Direction::Cs => self.cs.as_ref(),
        }
        .expect("ADU created")
    }

    fn adu_mut(&mut self, direction: Direction) -> &mut Adu {
        match direction {
            Direction::Sc => self.sc.as_mut(),
            Direction::Cs => self.cs.as_mut(),
        }
        .expect("ADU created")
    }

    fn decode_adu(
        &mut self,
        ctx: &mut dyn Context,
        ss: u64,
        es: u64,
        ptype: &str,
        item: &Item,
        direction: Direction,
        opts: &Opts,
    ) -> Result<()> {
        // We don't have a nice way to get the baud rate from UART, so we
        // figure out how long a bit lasts from the start/stop bit.
        if self.bitlength.is_none() {
            if ptype == "STARTBIT" || ptype == "STOPBIT" {
                self.bitlength = Some(es - ss);
            } else {
                // If we don't know the bitlength yet, we can't start decoding.
                return Ok(());
            }
        }
        let bitlength = self.bitlength.expect("set above");

        // Select the ADU, create it if needed.
        let start_new_frame = match direction {
            Direction::Sc => self.sc.as_ref().is_none_or(|a| a.start_new_frame),
            Direction::Cs => self.cs.as_ref().is_none_or(|a| a.start_new_frame),
        };
        if start_new_frame {
            let base = match direction {
                Direction::Sc => SC_BASE,
                Direction::Cs => CS_BASE,
            };
            let adu = Adu::new(ss, base);
            match direction {
                Direction::Sc => self.sc = Some(adu),
                Direction::Cs => self.cs = Some(adu),
            }
        }

        // 3.5 characters of space end a frame; (3.5 + 1.5) / 2 * 11 ~= 28.
        let delta = i128::from(ss) - i128::from(self.adu(direction).last_read);
        let gap = i128::from(bitlength) * i128::from(opts.framegap);
        if delta <= gap {
            let adu = self.adu_mut(direction);
            adu.add_data(ctx, ss, es, ptype, item)
        } else {
            // It's been too long since the last part of the ADU.
            let last_end = self.adu(direction).data.last().map(|b| b.end);
            if let Some(end) = last_end {
                // Extend errors for 3 bits after the last byte.
                let message_overflow = end + bitlength * 3;
                let adu = self.adu_mut(direction);
                if let Err(Flow::Hard(message)) = adu.close(ctx, opts, message_overflow) {
                    return Err(Error::arg(message));
                }
            }
            self.adu_mut(direction).start_new_frame = true;
            // Restart this function, it will make a new ADU for us.
            self.decode_adu(ctx, ss, es, ptype, item, direction, opts)
        }
    }
}

/// Unpacks `[ptype, rxtx, pdata]` and returns the packet type and
/// direction, as the UART output tuple.
fn unpack_uart(item: &Item) -> Result<(&str, i64)> {
    let Item::List(items) = item else {
        return Err(Error::arg("cannot unpack non-sequence UART item"));
    };
    if items.len() != 3 {
        return Err(Error::arg("too many values to unpack (UART item)"));
    }
    let Item::Str(ptype) = &items[0] else {
        return Err(Error::arg("UART packet type is not a string"));
    };
    match items[1] {
        Item::Int(rxtx) => Ok((ptype, rxtx)),
        _ => Err(Error::arg("UART direction is not an integer")),
    }
}

/// `pdata[0]`: the byte value of a DATA item.
fn data_value(item: &Item) -> Result<u8> {
    let Item::List(items) = item else {
        return Err(Error::arg("cannot unpack non-sequence UART item"));
    };
    if items.len() != 3 {
        return Err(Error::arg("too many values to unpack (UART item)"));
    }
    let Item::List(pdata) = &items[2] else {
        return Err(Error::arg("UART data is not a sequence"));
    };
    match pdata.first() {
        Some(Item::Int(v)) => u8::try_from(*v).map_err(|_| Error::arg("byte out of range")),
        _ => Err(Error::arg("UART data has no byte value")),
    }
}

/// Python's `str.format(value)` for the templates `putl` receives:
/// `{}`, `{0}` and the padding specs `:08b`, `:X`, `:02X`, `:02x`, `:d`.
fn format_last(template: &str, value: u8) -> PResult<String> {
    let mut out = String::new();
    let mut chars = template.chars().peekable();
    while let Some(c) = chars.next() {
        match c {
            '{' => {
                if chars.peek() == Some(&'{') {
                    chars.next();
                    out.push('{');
                    continue;
                }
                let mut spec = String::new();
                let mut closed = false;
                for c in chars.by_ref() {
                    if c == '}' {
                        closed = true;
                        break;
                    }
                    spec.push(c);
                }
                if !closed {
                    return hard("Single '{' encountered in format string");
                }
                let (field, fmt) = match spec.split_once(':') {
                    Some((field, fmt)) => (field, fmt),
                    None => (spec.as_str(), ""),
                };
                if !field.chars().all(|c| c.is_ascii_digit()) {
                    return hard("invalid format field");
                }
                if field.parse::<usize>().is_ok_and(|index| index != 0) {
                    return hard("index out of range");
                }
                out.push_str(&format_value(i64::from(value), fmt)?);
            }
            '}' => {
                if chars.peek() == Some(&'}') {
                    chars.next();
                    out.push('}');
                    continue;
                }
                return hard("Single '}' encountered in format string");
            }
            c => out.push(c),
        }
    }
    Ok(out)
}

fn format_value(value: i64, spec: &str) -> PResult<String> {
    if spec.is_empty() || spec == "d" {
        return Ok(value.to_string());
    }
    let ty = spec.chars().last().unwrap_or('d');
    let width: usize = spec[..spec.len() - 1].parse().unwrap_or(0);
    Ok(match ty {
        'b' => format!("{value:0width$b}"),
        'x' => format!("{value:0width$x}"),
        'X' => format!("{value:0width$X}"),
        'o' => format!("{value:0width$o}"),
        'd' => format!("{value:0width$}"),
        _ => return hard("Unknown format code"),
    })
}

impl Decoder for Modbus {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, config: &Config) -> Result<()> {
        *self = Modbus {
            opts: Some(Opts {
                scchannel: config.str("scchannel", "RX").to_owned(),
                cschannel: config.str("cschannel", "TX").to_owned(),
                framegap: config.int("framegap", 28),
            }),
            sc: None,
            cs: None,
            bitlength: None,
        };
        Ok(())
    }

    fn decode(&mut self, _ctx: &mut dyn Context) -> Result<()> {
        Err(Error::not_supported("modbus is a stacked decoder"))
    }

    fn decode_item(&mut self, ss: u64, es: u64, item: &Item, ctx: &mut dyn Context) -> Result<()> {
        let (ptype, rxtx) = unpack_uart(item)?;

        // Ignore unknown/unsupported ptypes.
        if !matches!(ptype, "STARTBIT" | "DATA" | "STOPBIT") {
            return Ok(());
        }

        let top = usize::try_from(rxtx)
            .ok()
            .and_then(|i| RXTX_CHANNELS.get(i))
            .copied()
            .ok_or_else(|| Error::arg("list index out of range"))?;
        let opts = self.opts.clone().ok_or_else(|| Error::arg("decoder not started"))?;

        // A packet may go to both ADUs.
        if top == opts.scchannel {
            self.decode_adu(ctx, ss, es, ptype, item, Direction::Sc, &opts)?;
        }
        if top == opts.cschannel {
            self.decode_adu(ctx, ss, es, ptype, item, Direction::Cs, &opts)?;
        }
        Ok(())
    }
}
