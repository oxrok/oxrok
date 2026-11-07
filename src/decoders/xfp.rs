// Counterpart in sigrok:
//   libsigrokdecode/decoders/xfp/pd.py and common/plugtrx/mod.py,
//   Copyright (C) 2013 Bert Vermeulen <bert@biot.com>,
//   Copyright (C) 2016 Bert Vermeulen <bert@biot.com>.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the I2C bus protocol and the decoder's documented classes and texts.

//! XFP I²C management interface structures.
//!
//! Upstream keeps a single byte counter across the whole capture and
//! feeds the accumulated bytes of a structure field to the handler whose
//! key is that field's last byte index. The lower memory map (0x00..0x7f)
//! is decoded unconditionally; the upper map (0x80..0xff) only when the
//! page select byte in lower memory says 0x01 (table 0x01, the serial ID
//! table). `/0x7f` (page select) and `/0x01` (extended module id) have no
//! annotation of their own.

use crate::core::{Error, Result};
use crate::decode::{AnnClass, Config, Context, Decoder, DecoderDef, DecoderInfo, Item};

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "xfp",
    name: "XFP",
    longname: "10 Gigabit Small Form Factor Pluggable Module (XFP)",
    desc: "XFP I²C management interface structures/protocol",
    license: "gplv3+",
    inputs: &["i2c"],
    outputs: &[],
    tags: &["Networking"],
    channels: &[],
    optional_channels: &[],
    options: &[],
    annotations: &[
        AnnClass { id: "fieldnames-and-values", desc: "XFP structure field names and values" },
        AnnClass { id: "fields", desc: "XFP structure fields" },
    ],
    annotation_rows: &[],
    binary: &[],
    // The doc is the package's __init__.py docstring, which the Python
    // host reads from the imported module.
    doc: "This PD decodes the XFP I²C management interface structures/protocol.\n\
\n\
XFP modules include an I²C interface, used to monitor and control various\n\
aspects of the module. The specification defines an I²C slave at address\n\
0x50 (0xa0) which returns 128 bytes of a standard structure (\"lower memory\"),\n\
and, after setting a table number in lower memory, a set of 256 \"higher\n\
memory\" tables, which can be mapped to different subdevices on the XFP.\n\
\n\
Only one table is defined in the specification: table 0x01, the default on\n\
module startup. Other table are either reserved for future expansion, or\n\
available for vendor-specific extensions. This decoder supports both lower\n\
memory and table 0x01.\n\
\n\
Details:\n\
ftp://ftp.seagate.com/sff/INF-8077.PDF (XFP specification)",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(Xfp::default())
}

/// `MODULE_ID`.
fn module_name(v: i64) -> &'static str {
    match v {
        0x01 => "GBIC",
        0x02 => "Integrated module/connector",
        0x03 => "SFP",
        0x04 => "300-pin XBI",
        0x05 => "XENPAK",
        0x06 => "XFP",
        0x07 => "XFF",
        0x08 => "XFP-E",
        0x09 => "XPAK",
        0x0a => "X2",
        _ => "Unknown",
    }
}

/// `ALARM_THRESHOLDS`, keyed by the field's first byte index.
fn alarm_threshold(idx: i64) -> &'static str {
    match idx {
        0 => "Temp high alarm",
        2 => "Temp low alarm",
        4 => "Temp high warning",
        6 => "Temp low warning",
        16 => "Bias high alarm",
        18 => "Bias low alarm",
        20 => "Bias high warning",
        22 => "Bias low warning",
        24 => "TX power high alarm",
        26 => "TX power low alarm",
        28 => "TX power high warning",
        30 => "TX power low warning",
        32 => "RX power high alarm",
        34 => "RX power low alarm",
        36 => "RX power high warning",
        38 => "RX power low warning",
        40 => "AUX 1 high alarm",
        42 => "AUX 1 low alarm",
        44 => "AUX 1 high warning",
        46 => "AUX 1 low warning",
        48 => "AUX 2 high alarm",
        50 => "AUX 2 low alarm",
        52 => "AUX 2 high warning",
        54 => "AUX 2 low warning",
        _ => "...",
    }
}

/// `AD_READOUTS`, keyed by the field's first byte index.
fn ad_readout_name(idx: i64) -> &'static str {
    match idx {
        0 => "Module temperature",
        4 => "TX bias current",
        6 => "Measured TX output power",
        8 => "Measured RX input power",
        10 => "AUX 1 measurement",
        12 => "AUX 2 measurement",
        _ => "...",
    }
}

/// `GCS_BITS`, in the order the bit field is walked.
const GCS_BITS: [&str; 13] = [
    "TX disable",
    "Soft TX disable",
    "MOD_NR",
    "P_Down",
    "Soft P_Down",
    "Interrupt",
    "RX_LOS",
    "Data_Not_Ready",
    "TX_NR",
    "TX_Fault",
    "TX_CDR not locked",
    "RX_NR",
    "RX_CDR not locked",
];

/// `CONNECTOR`.
fn connector_name(v: i64) -> Option<&'static str> {
    Some(match v {
        0x01 => "SC",
        0x02 => "Fibre Channel style 1 copper",
        0x03 => "Fibre Channel style 2 copper",
        0x04 => "BNC/TNC",
        0x05 => "Fibre Channel coax",
        0x06 => "FiberJack",
        0x07 => "LC",
        0x08 => "MT-RJ",
        0x09 => "MU",
        0x0a => "SG",
        0x0b => "Optical pigtail",
        0x20 => "HSSDC II",
        0x21 => "Copper pigtail",
        _ => return None,
    })
}

/// `TRANSCEIVER`, eight bit fields of differing lengths.
const TRANSCEIVER: [&[&str]; 8] = [
    &[
        "10GBASE-SR",
        "10GBASE-LR",
        "10GBASE-ER",
        "10GBASE-LRM",
        "10GBASE-SW",
        "10GBASE-LW",
        "10GBASE-EW",
    ],
    &["1200-MX-SN-I", "1200-SM-LL-L", "Extended Reach 1550 nm", "Intermediate reach 1300 nm FP"],
    &[],
    &[
        "1000BASE-SX / 1xFC MMF",
        "1000BASE-LX / 1xFC SMF",
        "2xFC MMF",
        "2xFC SMF",
        "OC48-SR",
        "OC48-IR",
        "OC48-LR",
    ],
    &["I-64.1r", "I-64.1", "I-64.2r", "I-64.2", "I-64.3", "I-64.5"],
    &["S-64.1", "S-64.2a", "S-64.2b", "S-64.3a", "S-64.3b", "S-64.5a", "S-64.5b"],
    &["L-64.1", "L-64.2a", "L-64.2b", "L-64.2c", "L-64.3", "G.959.1 P1L1-2D2"],
    &["V-64.2a", "V-64.2b", "V-64.3"],
];

/// `SERIAL_ENCODING`.
const SERIAL_ENCODING: [&str; 5] = ["64B/66B", "8B/10B", "SONET scrambled", "NRZ", "RZ"];

/// `XMIT_TECH`. Upstream's list is missing two commas, so the last
/// three literals are one concatenated string; that is reproduced here.
const XMIT_TECH: [&str; 7] = [
    "850 nm VCSEL",
    "1310 nm VCSEL",
    "1550 nm VCSEL",
    "1310 nm FP",
    "1310 nm DFB",
    "1550 nm DFB",
    "1310 nm EML1550 nm EMLcopper",
];

/// `CDR`.
const CDR: [&str; 8] = [
    "9.95Gb/s",
    "10.3Gb/s",
    "10.5Gb/s",
    "10.7Gb/s",
    "11.1Gb/s",
    "(unknown)",
    "lineside loopback mode",
    "XFI loopback mode",
];

/// `DEVICE_TECH`, including upstream's `sctive` typo.
const DEVICE_TECH: [&[&str]; 4] = [
    &["no wavelength control", "sctive wavelength control"],
    &["uncooled transmitter device", "cooled transmitter"],
    &["PIN detector", "APD detector"],
    &["transmitter not tunable", "transmitter tunable"],
];

/// `ENHANCED_OPTS`.
const ENHANCED_OPTS: [&str; 8] = [
    "VPS",
    "soft TX_DISABLE",
    "soft P_Down",
    "VPS LV regulator mode",
    "VPS bypassed regulator mode",
    "active FEC control",
    "wavelength tunability",
    "CMU",
];

/// `AUX_TYPES`.
const AUX_TYPES: [&str; 16] = [
    "not implemented",
    "APD bias voltage",
    "(unknown)",
    "TEC current",
    "laser temperature",
    "laser wavelength",
    "5V supply voltage",
    "3.3V supply voltage",
    "1.8V supply voltage",
    "-5.2V supply voltage",
    "5V supply current",
    "(unknown)",
    "(unknown)",
    "3.3V supply current",
    "1.8V supply current",
    "-5.2V supply current",
];

/// The XFP decoder.
pub struct Xfp {
    /// Received data items, used as an index into samplenum/data.
    cnt: i64,
    /// Start/end sample numbers per data item.
    sn: Vec<(u64, u64)>,
    /// Multi-byte structure buffer.
    buf: Vec<i64>,
    /// Filled in by address 0x7f in low memory.
    cur_highmem_page: i64,
    /// Filled in by extended ID value in table 2. Upstream never reads
    /// it again, but keeps it.
    #[allow(dead_code)]
    have_clei: bool,
}

impl Default for Xfp {
    fn default() -> Self {
        Xfp { cnt: -1, sn: Vec::new(), buf: Vec::new(), cur_highmem_page: 0, have_clei: false }
    }
}

/// An I2C packet is `[<ptype>, <pdata>]`.
fn split_packet(item: &Item) -> Result<(&str, &Item)> {
    let Item::List(parts) = item else {
        return Err(Error::device("xfp: expected an I2C packet"));
    };
    if parts.len() != 2 {
        return Err(Error::device("xfp: expected a two element I2C packet"));
    }
    let Item::Str(cmd) = &parts[0] else {
        return Err(Error::device("xfp: packet type is not a string"));
    };
    Ok((cmd.as_str(), &parts[1]))
}

/// Python's `str()` of a list of ints, e.g. `[1, 2]`.
fn py_int_list(data: &[i64]) -> String {
    let parts: Vec<String> = data.iter().map(i64::to_string).collect();
    format!("[{}]", parts.join(", "))
}

/// Python's `bytes(data).strip().decode('ascii').strip('\x00')`, where a
/// decode error stops the decoder as it does upstream.
fn ascii_field(data: &[i64]) -> Result<String> {
    let mut bytes = Vec::with_capacity(data.len());
    for &b in data {
        if !(0..=0x7f).contains(&b) {
            return Err(Error::device("xfp: field is not ASCII"));
        }
        bytes.push(b as u8);
    }
    let text = String::from_utf8(bytes).map_err(|_| Error::device("xfp: field is not ASCII"))?;
    // bytes.strip() strips ASCII whitespace from both ends.
    let text = text.trim_matches(|c: char| c.is_ascii_whitespace());
    Ok(text.trim_matches('\0').to_owned())
}

impl Xfp {
    /// Annotation helper: a name/value pair on row 0 and the bare value
    /// on row 1, over `start_cnt..=end_cnt` data items.
    fn annotate(
        &mut self,
        ctx: &mut dyn Context,
        key: &str,
        value: &str,
        start_cnt: Option<i64>,
        end_cnt: Option<i64>,
    ) -> Result<()> {
        let start = start_cnt.unwrap_or(self.cnt - self.buf.len() as i64 + 1);
        let end = end_cnt.unwrap_or(self.cnt);
        let (ss, _) = *self
            .sn
            .get(usize::try_from(start).unwrap_or(usize::MAX))
            .ok_or_else(|| Error::device("xfp: annotation start out of range"))?;
        let (_, es) = *self
            .sn
            .get(usize::try_from(end).unwrap_or(usize::MAX))
            .ok_or_else(|| Error::device("xfp: annotation end out of range"))?;
        ctx.put_ann(ss, es, 0, vec![format!("{key}: {value}")]);
        ctx.put_ann(ss, es, 1, vec![value.to_owned()]);
        Ok(())
    }

    /// Placeholder handler, needed to advance the buffer past unused or
    /// reserved space in the structures.
    fn ignore(&mut self) {}

    // Convert 16-bit two's complement values, with each increment
    // representing 1/256C, to degrees Celsius.
    fn to_temp(value: i64) -> String {
        let value = if value & 0x8000 != 0 { -((value ^ 0xffff) + 1) } else { value };
        format!("{:.1} C", value as f64 / 256.0)
    }

    // TX bias current in uA. Each increment represents 0.2uA.
    fn to_current(value: i64) -> String {
        format!("{:.1} mA", value as f64 / 500_000.0)
    }

    // Power in mW, with each increment representing 0.1uW.
    fn to_power(value: i64) -> String {
        format!("{:.2} mW", value as f64 / 10_000.0)
    }

    // Wavelength in increments of 0.05nm.
    fn to_wavelength(value: i64) -> String {
        // Python 3's `/` makes a float, then `%d` truncates it.
        format!("{} nm", value / 20)
    }

    // Wavelength in increments of 0.005nm.
    fn to_wavelength_tolerance(value: i64) -> String {
        format!("{:.1} nm", value as f64 / 200.0)
    }

    fn module_id(&mut self, ctx: &mut dyn Context) -> Result<()> {
        let v = self.buf[0];
        self.annotate(ctx, "Module identifier", module_name(v), None, None)
    }

    fn signal_cc(&mut self, ctx: &mut dyn Context) -> Result<()> {
        // No good data available.
        if self.buf[0] != 0x00 {
            let v = format!("{:02x}", self.buf[0]);
            self.annotate(ctx, "Signal Conditioner Control", &v, None, None)?;
        }
        Ok(())
    }

    fn alarm_warnings(&mut self, ctx: &mut dyn Context) -> Result<()> {
        let data = self.buf.clone();
        let cnt_idx = self.cnt - data.len() as i64;
        let mut idx = 0;
        while idx < 56 {
            if idx == 8 {
                // Skip over reserved A/D flag thresholds.
                idx += 8;
            }
            let value = (data[idx as usize] << 8) | data[idx as usize + 1];
            if value != 0 {
                let name = alarm_threshold(idx);
                let start = cnt_idx + idx;
                let end = cnt_idx + idx + 1;
                if [0, 2, 4, 6].contains(&idx) {
                    self.annotate(ctx, name, &Self::to_temp(value), Some(start), Some(end))?;
                } else if [16, 18, 20, 22].contains(&idx) {
                    self.annotate(ctx, name, &Self::to_current(value), Some(start), Some(end))?;
                } else if [24, 26, 28, 30, 32, 34, 36, 38].contains(&idx) {
                    self.annotate(ctx, name, &Self::to_power(value), Some(start), Some(end))?;
                } else {
                    // Upstream calls `self.annotate(name, "%d" % name, ...)`,
                    // which raises a TypeError before it can emit anything.
                    return Err(Error::device("xfp: unsupported A/D alarm threshold formatting"));
                }
            }
            idx += 2;
        }
        Ok(())
    }

    fn vps(&mut self, ctx: &mut dyn Context) -> Result<()> {
        // No good data available.
        if self.buf != [0, 0] {
            let v = format!("{:02x}{:02x}", self.buf[0], self.buf[1]);
            self.annotate(ctx, "VPS", &v, None, None)?;
        }
        Ok(())
    }

    fn ber(&mut self, ctx: &mut dyn Context) -> Result<()> {
        // No good data available.
        if self.buf != [0, 0] {
            let v = py_int_list(&self.buf.clone());
            self.annotate(ctx, "BER", &v, None, None)?;
        }
        Ok(())
    }

    fn wavelength_cr(&mut self, ctx: &mut dyn Context) -> Result<()> {
        // No good data available.
        if self.buf != [0, 0, 0, 0] {
            let v = py_int_list(&self.buf.clone());
            self.annotate(ctx, "WCR", &v, None, None)?;
        }
        Ok(())
    }

    fn fec_cr(&mut self, ctx: &mut dyn Context) -> Result<()> {
        if self.buf != [0, 0, 0, 0] {
            let v = py_int_list(&self.buf.clone());
            self.annotate(ctx, "FEC", &v, None, None)?;
        }
        Ok(())
    }

    fn int_ctrl(&mut self, ctx: &mut dyn Context) -> Result<()> {
        // No good data available. Also boring.
        let out: Vec<String> = self.buf.iter().map(|d| format!("{d:02x}")).collect();
        self.annotate(ctx, "Interrupt bits", &out.join(" "), None, None)
    }

    fn ad_readout(&mut self, ctx: &mut dyn Context) -> Result<()> {
        let data = self.buf.clone();
        let cnt_idx = self.cnt - data.len() as i64 + 1;
        let mut idx = 0;
        while idx < 14 {
            if idx == 2 {
                // Skip over reserved field.
                idx += 2;
            }
            let value = (data[idx as usize] << 8) | data[idx as usize + 1];
            let name = ad_readout_name(idx);
            if value != 0 {
                let start = cnt_idx + idx;
                let end = cnt_idx + idx + 1;
                if idx == 0 {
                    self.annotate(ctx, name, &Self::to_temp(value), Some(start), Some(end))?;
                } else if idx == 4 {
                    self.annotate(ctx, name, &Self::to_current(value), Some(start), Some(end))?;
                } else if idx == 6 || idx == 8 {
                    self.annotate(ctx, name, &Self::to_power(value), Some(start), Some(end))?;
                } else {
                    self.annotate(ctx, name, &value.to_string(), Some(start), Some(end))?;
                }
            }
            idx += 2;
        }
        Ok(())
    }

    fn gcs(&mut self, ctx: &mut dyn Context) -> Result<()> {
        let mut allbits = (self.buf[0] << 8) | self.buf[1];
        let mut out = Vec::new();
        for b in 0..13 {
            if allbits & 0x8000 != 0 {
                out.push(GCS_BITS[b]);
            }
            allbits <<= 1;
        }
        self.annotate(ctx, "General Control/Status", &out.join(", "), None, None)
    }

    fn page_select(&mut self) {
        self.cur_highmem_page = self.buf[0];
    }

    fn ext_module_id(&mut self, ctx: &mut dyn Context) -> Result<()> {
        let v = self.buf[0];
        let mut out = vec![format!("Power level {} module", (v >> 6) + 1)];
        if v & 0x20 == 0 {
            out.push("CDR".to_owned());
        }
        if v & 0x10 == 0 {
            out.push("TX ref clock input required".to_owned());
        }
        if v & 0x08 == 0 {
            self.have_clei = true;
        }
        self.annotate(ctx, "Extended id", &out.join(", "), None, None)
    }

    fn connector(&mut self, ctx: &mut dyn Context) -> Result<()> {
        if let Some(name) = connector_name(self.buf[0]) {
            self.annotate(ctx, "Connector", name, None, None)?;
        }
        Ok(())
    }

    fn transceiver(&mut self, ctx: &mut dyn Context) -> Result<()> {
        let data = self.buf.clone();
        let mut out = Vec::new();
        for t in 0..8 {
            if data[t] == 0 {
                continue;
            }
            let mut value = data[t];
            for b in 0..8 {
                if value & 0x80 != 0 {
                    if TRANSCEIVER[t].len() < b + 1 {
                        out.push("(unknown)".to_owned());
                    } else {
                        out.push(TRANSCEIVER[t][b].to_owned());
                    }
                }
                value <<= 1;
            }
        }
        self.annotate(ctx, "Transceiver compliance", &out.join(", "), None, None)
    }

    fn serial_encoding(&mut self, ctx: &mut dyn Context) -> Result<()> {
        let mut out = Vec::new();
        let mut value = self.buf[0];
        for b in 0..8 {
            if value & 0x80 != 0 {
                if SERIAL_ENCODING.len() < b + 1 {
                    out.push("(unknown)".to_owned());
                } else {
                    out.push(SERIAL_ENCODING[b].to_owned());
                }
                value <<= 1;
            }
        }
        self.annotate(ctx, "Serial encoding support", &out.join(", "), None, None)
    }

    fn br_min(&mut self, ctx: &mut dyn Context) -> Result<()> {
        // Increments represent 100Mb/s.
        let rate = self.buf[0] as f64 / 10.0;
        self.annotate(ctx, "Minimum bit rate", &format!("{rate:.3} GB/s"), None, None)
    }

    fn br_max(&mut self, ctx: &mut dyn Context) -> Result<()> {
        // Increments represent 100Mb/s.
        let rate = self.buf[0] as f64 / 10.0;
        self.annotate(ctx, "Maximum bit rate", &format!("{rate:.3} GB/s"), None, None)
    }

    fn link_length_smf(&mut self, ctx: &mut dyn Context) -> Result<()> {
        let length = match self.buf[0] {
            0 => "(standard)".to_owned(),
            255 => "> 254 km".to_owned(),
            v => format!("{v} km"),
        };
        self.annotate(ctx, "Link length (SMF)", &length, None, None)
    }

    fn link_length_e50(&mut self, ctx: &mut dyn Context) -> Result<()> {
        let length = match self.buf[0] {
            0 => "(standard)".to_owned(),
            255 => "> 508 m".to_owned(),
            v => format!("{} m", v * 2),
        };
        self.annotate(ctx, "Link length (extended, 50μm MMF)", &length, None, None)
    }

    fn link_length_50um(&mut self, ctx: &mut dyn Context) -> Result<()> {
        let length = match self.buf[0] {
            0 => "(standard)".to_owned(),
            255 => "> 254 m".to_owned(),
            v => format!("{v} m"),
        };
        self.annotate(ctx, "Link length (50μm MMF)", &length, None, None)
    }

    fn link_length_625um(&mut self, ctx: &mut dyn Context) -> Result<()> {
        let length = match self.buf[0] {
            0 => "(standard)".to_owned(),
            255 => "> 254 m".to_owned(),
            v => format!("{v} m"),
        };
        self.annotate(ctx, "Link length (62.5μm MMF)", &length, None, None)
    }

    fn link_length_copper(&mut self, ctx: &mut dyn Context) -> Result<()> {
        let length = match self.buf[0] {
            0 => "(unknown)".to_owned(),
            255 => "> 254 m".to_owned(),
            v => format!("{} m", v * 2),
        };
        self.annotate(ctx, "Link length (copper)", &length, None, None)
    }

    fn device_tech(&mut self, ctx: &mut dyn Context) -> Result<()> {
        let mut out = Vec::new();
        let v = self.buf[0];
        let xmit = v >> 4;
        if xmit <= XMIT_TECH.len() as i64 - 1 {
            out.push(format!("{} transmitter", XMIT_TECH[xmit as usize]));
        }
        let dev = v & 0x0f;
        for b in 0..4 {
            out.push(DEVICE_TECH[b][((dev >> (3 - b)) & 0x01) as usize].to_owned());
        }
        self.annotate(ctx, "Device technology", &out.join(", "), None, None)
    }

    fn vendor(&mut self, ctx: &mut dyn Context) -> Result<()> {
        let name = ascii_field(&self.buf.clone())?;
        if !name.is_empty() {
            self.annotate(ctx, "Vendor", &name, None, None)?;
        }
        Ok(())
    }

    fn cdr(&mut self, ctx: &mut dyn Context) -> Result<()> {
        let mut out = Vec::new();
        let mut value = self.buf[0];
        for b in 0..8 {
            if value & 0x80 != 0 {
                out.push(CDR[b].to_owned());
            }
            value <<= 1;
        }
        self.annotate(ctx, "CDR support", &out.join(", "), None, None)
    }

    fn vendor_oui(&mut self, ctx: &mut dyn Context) -> Result<()> {
        if self.buf != [0, 0, 0] {
            let v = format!("{:02X}-{:02X}-{:02X}", self.buf[0], self.buf[1], self.buf[2]);
            self.annotate(ctx, "Vendor OUI", &v, None, None)?;
        }
        Ok(())
    }

    fn vendor_pn(&mut self, ctx: &mut dyn Context) -> Result<()> {
        let name = ascii_field(&self.buf.clone())?;
        if !name.is_empty() {
            self.annotate(ctx, "Vendor part number", &name, None, None)?;
        }
        Ok(())
    }

    fn vendor_rev(&mut self, ctx: &mut dyn Context) -> Result<()> {
        let name = ascii_field(&self.buf.clone())?;
        if !name.is_empty() {
            self.annotate(ctx, "Vendor revision", &name, None, None)?;
        }
        Ok(())
    }

    fn wavelength(&mut self, ctx: &mut dyn Context) -> Result<()> {
        let value = (self.buf[0] << 8) | self.buf[1];
        self.annotate(ctx, "Wavelength", &Self::to_wavelength(value), None, None)
    }

    fn wavelength_tolerance(&mut self, ctx: &mut dyn Context) -> Result<()> {
        let value = (self.buf[0] << 8) | self.buf[1];
        self.annotate(
            ctx,
            "Wavelength tolerance",
            &Self::to_wavelength_tolerance(value),
            None,
            None,
        )
    }

    fn max_case_temp(&mut self, ctx: &mut dyn Context) -> Result<()> {
        self.annotate(ctx, "Maximum case temperature", &format!("{} C", self.buf[0]), None, None)
    }

    fn power_supply(&mut self, ctx: &mut dyn Context) -> Result<()> {
        let d = self.buf.clone();
        let cnt = self.cnt;
        self.annotate(
            ctx,
            "Max power dissipation",
            &format!("{:.3} W", d[0] as f64 * 0.02),
            Some(cnt - 3),
            Some(cnt - 3),
        )?;
        self.annotate(
            ctx,
            "Max power dissipation (powered down)",
            &format!("{:.3} W", d[1] as f64 * 0.01),
            Some(cnt - 2),
            Some(cnt - 2),
        )?;
        self.annotate(
            ctx,
            "Max current required (5V supply)",
            &format!("{:.3} A", (d[2] >> 4) as f64 * 0.050),
            Some(cnt - 1),
            Some(cnt - 1),
        )?;
        self.annotate(
            ctx,
            "Max current required (3.3V supply)",
            &format!("{:.3} A", (d[2] & 0x0f) as f64 * 0.100),
            Some(cnt - 1),
            Some(cnt - 1),
        )?;
        self.annotate(
            ctx,
            "Max current required (1.8V supply)",
            &format!("{:.3} A", (d[3] >> 4) as f64 * 0.100),
            Some(cnt),
            Some(cnt),
        )?;
        self.annotate(
            ctx,
            "Max current required (-5.2V supply)",
            &format!("{:.3} A", (d[3] & 0x0f) as f64 * 0.050),
            Some(cnt),
            Some(cnt),
        )
    }

    fn vendor_sn(&mut self, ctx: &mut dyn Context) -> Result<()> {
        let name = ascii_field(&self.buf.clone())?;
        if !name.is_empty() {
            self.annotate(ctx, "Vendor serial number", &name, None, None)?;
        }
        Ok(())
    }

    fn manuf_date(&mut self, ctx: &mut dyn Context) -> Result<()> {
        let data = self.buf.clone();
        let number = |range: std::ops::Range<usize>| -> Result<i64> {
            let text = ascii_field(&data[range])?;
            text.trim().parse::<i64>().map_err(|_| Error::device("xfp: bad manufacturing date"))
        };
        let y = number(0..2)? + 2000;
        let m = number(2..4)?;
        let d = number(4..6)?;
        let mut mnf = format!("{y:04}-{m:02}-{d:02}");
        let lot = ascii_field(&data[6..])?;
        if !lot.is_empty() {
            mnf.push_str(" lot ");
            mnf.push_str(&lot);
        }
        self.annotate(ctx, "Manufacturing date", &mnf, None, None)
    }

    fn diag_mon(&mut self, ctx: &mut dyn Context) -> Result<()> {
        let mut out = Vec::new();
        if self.buf[0] & 0x10 != 0 {
            out.push("BER support".to_owned());
        } else {
            out.push("no BER support".to_owned());
        }
        if self.buf[0] & 0x08 != 0 {
            out.push("average power measurement".to_owned());
        } else {
            out.push("OMA power measurement".to_owned());
        }
        self.annotate(ctx, "Diagnostic monitoring", &out.join(", "), None, None)
    }

    fn enhanced_opts(&mut self, ctx: &mut dyn Context) -> Result<()> {
        let mut out = Vec::new();
        let mut value = self.buf[0];
        for b in 0..8 {
            if value & 0x80 != 0 {
                out.push(ENHANCED_OPTS[b].to_owned());
            }
            value <<= 1;
        }
        self.annotate(ctx, "Enhanced option support", &out.join(", "), None, None)
    }

    fn aux_mon(&mut self, ctx: &mut dyn Context) -> Result<()> {
        let v = self.buf[0];
        let aux = AUX_TYPES[(v >> 4) as usize];
        self.annotate(ctx, "AUX1 monitoring", aux, None, None)?;
        let aux = AUX_TYPES[(v & 0x0f) as usize];
        self.annotate(ctx, "AUX2 monitoring", aux, None, None)
    }

    /// Show as ASCII if possible. Upstream reuses the field's first data
    /// item index for every printable byte, so all of them land on that
    /// one sample range; reproduced here.
    fn maybe_ascii(&mut self, ctx: &mut dyn Context) -> Result<()> {
        let data = self.buf.clone();
        for &b in &data {
            if (0x20..0x7f).contains(&b) {
                let cnt = self.cnt - data.len() as i64 + 1;
                let ch = char::from_u32(b as u32).unwrap_or('\u{fffd}').to_string();
                self.annotate(ctx, "Vendor ID", &ch, Some(cnt), Some(cnt))?;
            }
        }
        Ok(())
    }

    /// The handler whose key is `cnt` in the lower memory map.
    fn lower(&mut self, ctx: &mut dyn Context) -> Result<()> {
        match self.cnt {
            0 => self.module_id(ctx),
            1 => self.signal_cc(ctx),
            57 => self.alarm_warnings(ctx),
            59 => self.vps(ctx),
            69 => {
                self.ignore();
                Ok(())
            }
            71 => self.ber(ctx),
            75 => self.wavelength_cr(ctx),
            79 => self.fec_cr(ctx),
            95 => self.int_ctrl(ctx),
            109 => self.ad_readout(ctx),
            111 => self.gcs(ctx),
            117 | 118 | 122 | 126 => {
                self.ignore();
                Ok(())
            }
            127 => {
                self.page_select();
                Ok(())
            }
            _ => Ok(()),
        }
    }

    /// The handler whose key is `cnt` in the high memory table 0x01.
    fn high(&mut self, ctx: &mut dyn Context) -> Result<()> {
        match self.cnt {
            128 => self.module_id(ctx),
            129 => self.ext_module_id(ctx),
            130 => self.connector(ctx),
            138 => self.transceiver(ctx),
            139 => self.serial_encoding(ctx),
            140 => self.br_min(ctx),
            141 => self.br_max(ctx),
            142 => self.link_length_smf(ctx),
            143 => self.link_length_e50(ctx),
            144 => self.link_length_50um(ctx),
            145 => self.link_length_625um(ctx),
            146 => self.link_length_copper(ctx),
            147 => self.device_tech(ctx),
            163 => self.vendor(ctx),
            164 => self.cdr(ctx),
            167 => self.vendor_oui(ctx),
            183 => self.vendor_pn(ctx),
            185 => self.vendor_rev(ctx),
            187 => self.wavelength(ctx),
            189 => self.wavelength_tolerance(ctx),
            190 => self.max_case_temp(ctx),
            191 => {
                self.ignore();
                Ok(())
            }
            195 => self.power_supply(ctx),
            211 => self.vendor_sn(ctx),
            219 => self.manuf_date(ctx),
            220 => self.diag_mon(ctx),
            221 => self.enhanced_opts(ctx),
            222 => self.aux_mon(ctx),
            223 => {
                self.ignore();
                Ok(())
            }
            255 => self.maybe_ascii(ctx),
            _ => Ok(()),
        }
    }
}

impl Decoder for Xfp {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, _config: &Config) -> Result<()> {
        *self = Xfp::default();
        Ok(())
    }

    fn decode(&mut self, _ctx: &mut dyn Context) -> Result<()> {
        Err(Error::not_supported("xfp takes an i2c decoder as input"))
    }

    fn decode_item(&mut self, ss: u64, es: u64, item: &Item, ctx: &mut dyn Context) -> Result<()> {
        let (cmd, data) = split_packet(item)?;

        // We only care about actual data bytes that are read (for now).
        if cmd != "DATA READ" {
            return Ok(());
        }
        let Item::Int(databyte) = data else {
            return Err(Error::device("xfp: DATA READ has no data byte"));
        };

        self.cnt += 1;
        self.sn.push((ss, es));
        self.buf.push(*databyte);
        if self.cnt < 0x80 {
            let key = self.cnt;
            let mut handled = true;
            match key {
                0 | 1 | 57 | 59 | 69 | 71 | 75 | 79 | 95 | 109 | 111 | 117 | 118 | 122 | 126
                | 127 => {}
                _ => handled = false,
            }
            if handled {
                self.lower(ctx)?;
                self.buf.clear();
            }
        } else if self.cnt < 0x0100 && self.cur_highmem_page == 0x01 {
            let key = self.cnt;
            let handled = matches!(
                key,
                128 | 129 | 130 | 138
                    ..=147
                        | 163
                        | 164
                        | 167
                        | 183
                        | 185
                        | 187
                        | 189
                        | 190
                        | 191
                        | 195
                        | 211
                        | 219
                        | 220
                        | 221
                        | 222
                        | 223
                        | 255
            );
            if handled {
                self.high(ctx)?;
                self.buf.clear();
            }
        }
        Ok(())
    }
}
