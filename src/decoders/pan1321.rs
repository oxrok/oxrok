// Counterpart in sigrok:
//   libsigrokdecode/decoders/pan1321/pd.py,
//   Copyright (C) 2012-2013 Uwe Hermann <uwe@hermann-uwe.de>.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the PAN1321 command and reply texts.
#![allow(clippy::format_push_string)]

//! PAN1321: the Panasonic Bluetooth SPP module's AT command set.

use crate::core::{Error, Result};
use crate::decode::{AnnClass, Config, Context, Decoder, DecoderDef, DecoderInfo, Item};

const RX: usize = 0;

const ANN_TEXT_VERBOSE: usize = 0;
const ANN_TEXT: usize = 1;
const ANN_WARNINGS: usize = 2;

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "pan1321",
    name: "PAN1321",
    longname: "Panasonic PAN1321",
    desc: "Bluetooth RF module with Serial Port Profile (SPP).",
    license: "gplv2+",
    inputs: &["uart"],
    outputs: &[],
    tags: &["Wireless/RF"],
    channels: &[],
    optional_channels: &[],
    options: &[],
    annotations: &[
        AnnClass { id: "text-verbose", desc: "Human-readable text (verbose)" },
        AnnClass { id: "text", desc: "Human-readable text" },
        AnnClass { id: "warnings", desc: "Human-readable warnings" },
    ],
    annotation_rows: &[],
    binary: &[],
    doc: "This decoder stacks on top of the 'uart' PD and decodes the Panasonic PAN1321\n\
Bluetooth module Serial Port Profile (SPP) protocol.",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(Pan1321::new())
}

struct Pan1321 {
    cmd: [String; 2],
    ss_block: u64,
    es_block: u64,
}

impl Pan1321 {
    fn new() -> Self {
        Pan1321 { cmd: [String::new(), String::new()], ss_block: 0, es_block: 0 }
    }

    fn putx(&self, ctx: &mut dyn Context, class: usize, texts: Vec<String>) {
        ctx.put_ann(self.ss_block, self.es_block, class, texts);
    }

    fn verbose_and_short(&self, ctx: &mut dyn Context, verbose: String, short: String) {
        self.putx(ctx, ANN_TEXT_VERBOSE, vec![verbose]);
        self.putx(ctx, ANN_TEXT, vec![short]);
    }

    fn warning(&self, ctx: &mut dyn Context, text: String) {
        self.putx(ctx, ANN_WARNINGS, vec![text]);
    }

    /// `handle_host_command`: one complete `AT+...` command from the host.
    fn handle_host_command(&self, ctx: &mut dyn Context, s: &str) -> Result<()> {
        if s.starts_with("AT+JAAC") {
            // AT+JAAC=<auto_accept> (0 or 1)
            let p = &s[s.find('=').map_or(s.len(), |i| i + 1)..];
            if p != "0" && p != "1" {
                self.warning(ctx, format!("Warning: Invalid JAAC parameter \"{p}\""));
                return Ok(());
            }
            let x = if p == "1" { "Auto" } else { "Don't auto" };
            self.verbose_and_short(
                ctx,
                format!("{x}-accept new connections"),
                format!("{x}-accept connections"),
            );
        } else if s.starts_with("AT+JPRO") {
            // AT+JPRO=<mode> (0 or 1)
            let p = &s[s.find('=').map_or(s.len(), |i| i + 1)..];
            if p != "0" && p != "1" {
                self.warning(ctx, format!("Warning: Invalid JPRO parameter \"{p}\""));
                return Ok(());
            }
            let onoff = if p == "0" { "off" } else { "on" };
            let x = if p == "0" { "Leaving" } else { "Entering" };
            self.verbose_and_short(
                ctx,
                format!("{x} production mode"),
                format!("Production mode = {onoff}"),
            );
        } else if s.starts_with("AT+JRES") {
            // AT+JRES
            if s != "AT+JRES" {
                // JRES has no params.
                self.warning(ctx, "Warning: Invalid JRES usage.".into());
                return Ok(());
            }
            self.verbose_and_short(ctx, "Triggering a software reset".into(), "Reset".into());
        } else if s.starts_with("AT+JSDA") {
            // AT+JSDA=<l>,<d> (l: length in bytes, d: data)
            let params = &s[s.find('=').map_or(s.len(), |i| i + 1)..];
            let Some((l, d)) = params.split_once(',') else {
                return Err(Error::arg("not enough values to unpack (expected 2)"));
            };
            if d.contains(',') {
                return Err(Error::arg("too many values to unpack (expected 2)"));
            }
            if !l.chars().all(char::is_numeric) || l.is_empty() {
                self.warning(ctx, format!("Warning: Invalid data length \"{l}\"."));
            }
            let len = l.parse::<i64>().map_err(|_| Error::arg("invalid literal"))?;
            let dchars: Vec<char> = d.chars().collect();
            if len != dchars.len() as i64 {
                self.warning(
                    ctx,
                    format!("Warning: Data length mismatch ({} != {}).", len, dchars.len()),
                );
            }
            let mut b = String::new();
            for c in &dchars {
                b.push_str(&format!("{:02x} ", u32::from(*c)));
            }
            b.pop();
            self.verbose_and_short(
                ctx,
                format!("Sending {len} data bytes: {b}"),
                format!("Send {len} = {b}"),
            );
        } else if s.starts_with("AT+JSEC") {
            // AT+JSEC=<secmode>,<linkkey_info>,<pintype>,<pinlen>,<pin>
            let pin = last_chars(s, 4);
            self.verbose_and_short(
                ctx,
                format!("Host set the Bluetooth PIN to \"{pin}\""),
                format!("PIN = {pin}"),
            );
        } else if s.starts_with("AT+JSLN") {
            // AT+JSLN=<namelen>,<name>
            let name = &s[s.find(',').map_or(s.len(), |i| i + 1)..];
            self.verbose_and_short(
                ctx,
                format!("Host set the Bluetooth name to \"{name}\""),
                format!("BT name = {name}"),
            );
        } else {
            self.verbose_and_short(
                ctx,
                format!("Host sent unsupported command: {s}"),
                format!("Unsupported command: {s}"),
            );
        }
        Ok(())
    }

    /// `handle_device_reply`: one complete reply from the module.
    fn handle_device_reply(&self, ctx: &mut dyn Context, s: &str) {
        if s == "ROK" {
            self.verbose_and_short(ctx, "Device initialized correctly".into(), "Init".into());
        } else if s == "OK" {
            self.verbose_and_short(ctx, "Device acknowledged last command".into(), "ACK".into());
        } else if s.starts_with("ERR") {
            let error = &s[s.find('=').map_or(s.len(), |i| i + 1)..];
            self.verbose_and_short(
                ctx,
                format!("Device sent error code {error}"),
                format!("ERR = {error}"),
            );
        } else {
            self.verbose_and_short(
                ctx,
                format!("Device sent an unknown reply: {s}"),
                format!("Unknown reply: {s}"),
            );
        }
    }
}

/// The last `n` characters of `s`, as Python's `s[-n:]`.
fn last_chars(s: &str, n: usize) -> String {
    let chars: Vec<char> = s.chars().collect();
    chars[chars.len().saturating_sub(n)..].iter().collect()
}

impl Decoder for Pan1321 {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, _config: &Config) -> Result<()> {
        *self = Pan1321::new();
        Ok(())
    }

    fn decode(&mut self, _ctx: &mut dyn Context) -> Result<()> {
        Err(Error::not_supported("pan1321 is a stacked decoder"))
    }

    fn decode_item(&mut self, ss: u64, es: u64, item: &Item, ctx: &mut dyn Context) -> Result<()> {
        let (ptype, rxtx, pdata) = unpack_uart(item)?;

        // For now, ignore all UART packets except the actual data packets.
        if ptype != "DATA" {
            return Ok(());
        }
        let pdata = data_value(pdata)?;
        let rxtx = usize::try_from(rxtx).map_err(|_| Error::arg("list index out of range"))?;
        if rxtx >= 2 {
            return Err(Error::arg("list index out of range"));
        }

        // If this is the start of a command/reply, remember the start sample.
        if self.cmd[rxtx].is_empty() {
            self.ss_block = ss;
        }

        // Append a new (ASCII) byte to the currently built/parsed command.
        let c = char::from_u32(u32::try_from(pdata).map_err(|_| Error::arg("chr() arg"))?)
            .ok_or_else(|| Error::arg("chr() arg not in range"))?;
        self.cmd[rxtx].push(c);

        // Get packets/bytes until an \r\n sequence is found (end of command).
        if !self.cmd[rxtx].ends_with("\r\n") {
            return Ok(());
        }

        // Handle host commands and device replies; remove trailing \r\n.
        self.es_block = es;
        let s = self.cmd[rxtx][..self.cmd[rxtx].len() - 2].to_owned();
        let r = if rxtx == RX {
            self.handle_device_reply(ctx, &s);
            Ok(())
        } else {
            self.handle_host_command(ctx, &s)
        };

        self.cmd[rxtx].clear();
        r
    }
}

/// Unpacks `[ptype, rxtx, pdata]`; the UART output tuple.
fn unpack_uart(item: &Item) -> Result<(&str, i64, &Item)> {
    let Item::List(items) = item else {
        return Err(Error::arg("cannot unpack non-sequence UART item"));
    };
    if items.len() != 3 {
        return Err(Error::arg("too many values to unpack (UART item)"));
    }
    let Item::Str(ptype) = &items[0] else {
        return Err(Error::arg("UART packet type is not a string"));
    };
    let Item::Int(rxtx) = items[1] else {
        return Err(Error::arg("UART direction is not an integer"));
    };
    Ok((ptype, rxtx, &items[2]))
}

/// `pdata[0]`: the byte value of a DATA item.
fn data_value(pdata: &Item) -> Result<i64> {
    let Item::List(items) = pdata else {
        return Err(Error::arg("UART data is not a sequence"));
    };
    match items.first() {
        Some(Item::Int(v)) => Ok(*v),
        _ => Err(Error::arg("UART data has no byte value")),
    }
}
