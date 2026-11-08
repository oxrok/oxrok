// Counterpart in sigrok:
//   libsigrokdecode/decoders/ook_oregon/pd.py,
//   Copyright (C) 2018 Steve R <steversig@virginmedia.com>,
//   GPL-2.0-or-later.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the Oregon Scientific protocol notes and the
//   decoder's documented classes, fields and texts.
//
// Subtle points of the Python original that this port keeps:
//   - The OOK bit string is searched for a preamble marker anywhere in
//     its first 40 (v2.1), 17 (v1) or 28 (v3) characters; the marker is
//     split off at its first occurrence and the preamble length is that
//     occurrence. The v2.1 sensor is then halved into the v3 format.
//   - Nibbles are reversed (`nibble[::-1]`) before the hex text and the
//     field value are computed, so a field is read right to left in
//     four-bit groups while the whole field's hex value is the
//     concatenation of those reversed groups.
//   - `oregon_put_nib()` stores the label with its ": " suffix, and
//     uses `int(decode_pos / 4)` for unlabelled nibbles.
//   - A v2.1 checksum is 10 less than the sum, and the sensor specific
//     checksum versions (1D20, 5D60, EC40) override `self.ver`.
//   - `hex(checksum)[2:]` of a negative -1 checksum is "x1", which is
//     what a checksum aborted by a bit error prints.
//   - `%g` is Python's six significant digit format.

//! Oregon Scientific weather sensors on top of the OOK decoder.

use crate::core::{Error, Result};
use crate::decode::{
    AnnClass, AnnRow, BinClass, Config, Context, Decoder, DecoderDef, DecoderInfo, Item,
    OptionDesc, OptionValue,
};

const ANN_FIELD: usize = 1;
const ANN_L2: usize = 2;
const BIN_DATA_HEX: usize = 0;

static OPTIONS: [OptionDesc; 1] = [OptionDesc {
    id: "unknown",
    desc: "Unknown type is",
    default: OptionValue::s("Unknown"),
    values: &[
        OptionValue::s("Unknown"),
        OptionValue::s("Temp"),
        OptionValue::s("Temp_Hum"),
        OptionValue::s("Temp_Hum1"),
        OptionValue::s("Temp_Hum_Baro"),
        OptionValue::s("Temp_Hum_Baro1"),
        OptionValue::s("UV"),
        OptionValue::s("UV1"),
        OptionValue::s("Wind"),
        OptionValue::s("Rain"),
        OptionValue::s("Rain1"),
    ],
}];

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "ook_oregon",
    name: "Oregon",
    longname: "Oregon Scientific",
    desc: "Oregon Scientific weather sensor protocol.",
    license: "gplv2+",
    inputs: &["ook"],
    outputs: &[],
    tags: &["Sensor"],
    channels: &[],
    optional_channels: &[],
    options: &OPTIONS,
    annotations: &[
        AnnClass { id: "bit", desc: "Bit" },
        AnnClass { id: "field", desc: "Field" },
        AnnClass { id: "l2", desc: "Level 2" },
        AnnClass { id: "pre", desc: "Preamble" },
        AnnClass { id: "syn", desc: "Sync" },
        AnnClass { id: "id", desc: "SensorID" },
        AnnClass { id: "ch", desc: "Channel" },
        AnnClass { id: "roll", desc: "Rolling code" },
        AnnClass { id: "f1", desc: "Flags1" },
    ],
    annotation_rows: &[
        AnnRow { id: "bits", desc: "Bits", classes: &[0] },
        AnnRow { id: "fields", desc: "Fields", classes: &[1, 3, 4] },
        AnnRow { id: "l2", desc: "Level 2", classes: &[2] },
    ],
    binary: &[BinClass { id: "data-hex", desc: "Hex data" }],
    // The package's __init__.py docstring, which the Python host reads.
    doc: "This decoder stacks on top of the 'ook' PD and decodes the Oregon Scientific\n\
          433MHz remote control protocol for weather sensors.",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(OokOregon::default())
}

/// `sensor` in lists.py: sensor id, model names, sensor type.
static SENSORS: [(&str, &[&str], &str); 25] = [
    ("1984", &["WGR800"], "Wind"),
    ("1994", &["WGR800"], "Wind"),
    ("1A2D", &["THGR228N"], "Temp_Hum1"),
    ("1A3D", &["THGR918"], ""),
    ("1D20", &["THGN123N", "THGR122NX", "THGN123N", "THGR228N"], "Temp_Hum"),
    ("1D30", &["THGN500", "THGN132N"], ""),
    ("2914", &["PCR800"], "Rain"),
    ("2A19", &["PCR800"], "Rain1"),
    ("2A1D", &["RGR918"], "Rain"),
    ("2D10", &["RGR968", "PGR968 "], "Rain1"),
    ("3A0D", &["STR918", "WGR918"], "Wind"),
    ("5A5D", &["BTHR918"], ""),
    ("5A6D", &["BTHR918N"], "Temp_Hum_Baro"),
    ("5D53", &["BTHGN129"], "Baro"),
    ("5D60", &["BTHR968"], "Temp_Hum_Baro"),
    ("C844", &["THWR800"], "Temp"),
    ("CC13", &["RTGR328N"], "Temp_Hum"),
    ("CC23", &["THGR328N"], "Temp_Hum"),
    ("CD39", &["RTHR328N"], "Temp"),
    ("D874", &["UVN800"], "UV1"),
    ("EA4C", &["THWR288A"], "Temp"),
    ("EC40", &["THN132N", "THR238NF"], "Temp"),
    ("EC70", &["UVR128"], "UV"),
    ("F824", &["THGN800", "THGN801", "THGR810"], "Temp_Hum"),
    ("F8B4", &["THGR810"], "Temp_Hum"),
];

/// `dir_table` in lists.py.
static DIR_TABLE: [&str; 17] = [
    "N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE", "S", "SSW", "SW", "WSW", "W", "WNW", "NW",
    "NNW", "N",
];

/// `sensor_checksum` in lists.py: ids whose checksum method is v3.
static SENSOR_CHECKSUM: [(&str, &str); 3] = [("1D20", "v3"), ("5D60", "v3"), ("EC40", "v3")];

/// One decoded OOK bit: `[ss, es, bit]`.
#[derive(Clone, Copy)]
struct OokBit {
    ss: u64,
    es: u64,
    state: char,
}

/// One nibble saved for the level 2 decoder.
#[derive(Clone)]
struct Nibble {
    ss: u64,
    es: u64,
    result: String,
}

/// A Python list index: negative counts from the end.
fn py_index(i: i64, len: usize) -> Option<usize> {
    let len = len as i64;
    let j = if i < 0 { len + i } else { i };
    (0..len).contains(&j).then_some(j as usize)
}

/// Python's `s[start:start + n]`, including its negative index rules.
fn py_slice(s: &str, start: i64, n: i64) -> String {
    let len = s.len() as i64;
    let norm = |x: i64| if x < 0 { (len + x).max(0) } else { x.min(len) };
    let a = norm(start);
    let b = norm(start + n);
    if b <= a { String::new() } else { s[a as usize..b as usize].to_owned() }
}

/// Python's `s[start:]`.
fn py_slice_from(s: &str, start: i64) -> String {
    let len = s.len() as i64;
    let a = if start < 0 { (len + start).max(0) } else { start.min(len) };
    s[a as usize..].to_owned()
}

/// Strip the zeros of a `%g` fixed representation.
fn trim_g(s: &str) -> String {
    if s.contains('.') {
        let s = s.trim_end_matches('0');
        s.trim_end_matches('.').to_owned()
    } else {
        s.to_owned()
    }
}

/// Python's `'%g' % x` (six significant digits).
fn py_g(x: f64) -> String {
    if x == 0.0 {
        return if x.is_sign_negative() { "-0".to_owned() } else { "0".to_owned() };
    }
    if !x.is_finite() {
        return format!("{x}");
    }
    let sci = format!("{x:.5e}");
    let (mantissa, exp_str) = sci.split_once('e').unwrap_or((&sci, "0"));
    let exp: i32 = exp_str.parse().unwrap_or(0);
    if exp < -4 || exp >= 6 {
        let m = trim_g(mantissa);
        let sign = if exp < 0 { '-' } else { '+' };
        return format!("{m}e{sign}{:02}", exp.abs());
    }
    let decimals = (5 - exp).max(0) as usize;
    trim_g(&format!("{x:.decimals$}"))
}

/// Python's `hex(x)[2:]`.
fn hex_py(x: i64) -> String {
    if x < 0 { format!("x{:x}", -x) } else { format!("{x:x}") }
}

/// The ook_oregon decoder.
#[derive(Default)]
pub struct OokOregon {
    decoded: Vec<OokBit>,
    decoded_nibbles: Vec<Nibble>,
    ookstring: String,
    decode_pos: i64,
    ss: i64,
    es: i64,
    ver: String,
    sensor_id: String,
    unknown: String,
}

impl OokOregon {
    fn putx(&self, ctx: &mut dyn Context, class: usize, texts: Vec<String>) {
        ctx.put_ann(self.ss.max(0) as u64, self.es.max(0) as u64, class, texts);
    }

    fn get_bit(&self, i: i64) -> Result<&OokBit> {
        let idx = py_index(i, self.decoded.len())
            .ok_or_else(|| Error::device("ook_oregon: OOK bit index out of range"))?;
        Ok(&self.decoded[idx])
    }

    /// pd.py `dump_oregon_hex()`.
    fn dump_oregon_hex(&self, ctx: &mut dyn Context, start: u64, finish: u64) {
        let mut hexstring = String::new();
        for nib in &self.decoded_nibbles {
            if nib.result.is_empty() {
                hexstring.push(' ');
            } else {
                hexstring.push_str(&nib.result);
            }
        }
        let s = format!("Oregon {} \"{}\"\n", self.ver, hexstring.to_uppercase());
        ctx.put_binary(start, finish, BIN_DATA_HEX, s.into_bytes());
    }

    /// pd.py `oregon_put_pre_and_sync()`.
    fn put_pre_and_sync(
        &mut self,
        ctx: &mut dyn Context,
        len_pream: usize,
        len_sync: usize,
        ver: &str,
    ) -> Result<()> {
        self.decode_pos = len_pream as i64;
        let first = self.get_bit(0)?.ss;
        let end = self.get_bit(self.decode_pos)?.ss;
        self.ss = first as i64;
        self.es = end as i64;
        self.putx(
            ctx,
            ANN_FIELD,
            vec![
                format!("Oregon {ver} Preamble"),
                format!("{ver} Preamble"),
                format!("{ver} Pre"),
                ver.to_owned(),
            ],
        );
        self.decode_pos += len_sync as i64;
        let start = self.get_bit(len_pream as i64)?.ss;
        let end = self.get_bit(self.decode_pos)?.ss;
        self.ss = start as i64;
        self.es = end as i64;
        self.putx(ctx, ANN_FIELD, vec!["Sync".to_owned(), "Syn".to_owned(), "S".to_owned()]);

        let cut = self.decode_pos as usize;
        self.decoded = self.decoded[cut..].to_vec();
        self.ookstring = py_slice_from(&self.ookstring, self.decode_pos);
        self.ver = ver.to_owned();
        Ok(())
    }

    /// pd.py `oregon_nibbles()`: each four bit group reversed.
    fn nibbles(s: &str) -> Vec<String> {
        let num_nibbles = s.len() / 4;
        (0..num_nibbles)
            .map(|i| {
                let group = &s[4 * i..4 * i + 4];
                group.chars().rev().collect()
            })
            .collect()
    }

    /// pd.py `oregon_put_nib()`.
    fn put_nib(
        &mut self,
        ctx: &mut dyn Context,
        label: &str,
        start: u64,
        finish: u64,
        numbits: usize,
    ) -> Result<()> {
        let param = py_slice(&self.ookstring, self.decode_pos, numbits as i64);
        let param = Self::nibbles(&param);
        let joined = param.concat();
        let result = if joined.contains('E') {
            String::new()
        } else {
            let v = i64::from_str_radix(&joined, 2)
                .map_err(|_| Error::device("ook_oregon: field is not binary"))?;
            let h = format!("{v:x}");
            let digits = numbits / 4;
            if h.len() < digits { format!("{}{h}", "0".repeat(digits - h.len())) } else { h }
        };

        let mut text = String::new();
        if !label.is_empty() {
            text.push_str(label);
            text.push_str(": ");
        }
        text.push_str(&result);
        self.ss = start as i64;
        self.es = finish as i64;
        self.putx(ctx, ANN_FIELD, vec![text, result]);

        for (i, group) in param.iter().enumerate() {
            let a = py_index(self.decode_pos + 4 * i as i64, self.decoded.len())
                .ok_or_else(|| Error::device("ook_oregon: nibble start out of range"))?;
            let b = py_index(self.decode_pos + 4 * i as i64 + 3, self.decoded.len())
                .ok_or_else(|| Error::device("ook_oregon: nibble end out of range"))?;
            let result = if group.contains('E') {
                String::new()
            } else {
                let v = i64::from_str_radix(group, 2)
                    .map_err(|_| Error::device("ook_oregon: nibble is not binary"))?;
                format!("{v:x}")
            };
            let (ss, es) = (self.decoded[a].ss, self.decoded[b].es);
            self.decoded_nibbles.push(Nibble { ss, es, result });
        }
        self.decode_pos += numbits as i64;
        Ok(())
    }

    /// pd.py `oregon_put_l2_param()`.
    fn put_l2_param(
        &mut self,
        ctx: &mut dyn Context,
        offset: usize,
        digits: usize,
        dec_point: usize,
        pre_label: &str,
        label: &str,
    ) -> Result<()> {
        let nib = &self.decoded_nibbles;
        let out_string: String =
            nib[offset..offset + digits].iter().map(|n| n.result.clone()).collect();
        let mut result = String::new();
        if out_string.len() == digits {
            let mut value = 0.0f64;
            for i in (1..=dec_point).rev() {
                let n = &nib[offset + dec_point - i].result;
                value += i64::from_str_radix(n, 16)
                    .map_err(|_| Error::device("ook_oregon: nibble is not hex"))?
                    as f64
                    / 10f64.powi(i as i32);
            }
            for i in dec_point..digits {
                let n = &nib[offset + i].result;
                value += i64::from_str_radix(n, 16)
                    .map_err(|_| Error::device("ook_oregon: nibble is not hex"))?
                    as f64
                    * 10f64.powi(i as i32 - dec_point as i32);
            }
            result = py_g(value);
        }
        let mut es = nib[offset + digits - 1].es;
        if label == "\u{2103}" {
            es = nib[offset + digits].es;
        }
        self.ss = nib[offset].ss as i64;
        self.es = es as i64;
        self.putx(ctx, ANN_L2, vec![format!("{pre_label}{result}{label}"), result]);
        Ok(())
    }

    /// pd.py `oregon_temp()`.
    fn temp(&mut self, ctx: &mut dyn Context, offset: usize) -> Result<()> {
        let n = &self.decoded_nibbles[offset + 3].result;
        let sign = if n.is_empty() {
            "?"
        } else {
            let v = i64::from_str_radix(n, 16)
                .map_err(|_| Error::device("ook_oregon: nibble is not hex"))?;
            if v != 0 { "-" } else { "+" }
        };
        self.put_l2_param(ctx, offset, 3, 1, sign, "\u{2103}")
    }

    /// pd.py `oregon_baro()`.
    fn baro(&mut self, ctx: &mut dyn Context, offset: usize) -> Result<()> {
        let nib = &self.decoded_nibbles;
        let mut baro = String::new();
        if !nib[offset + 2].result.is_empty()
            && !nib[offset + 1].result.is_empty()
            && !nib[offset].result.is_empty()
        {
            let s = format!("{}{}", nib[offset + 1].result, nib[offset].result);
            let v = i64::from_str_radix(&s, 16)
                .map_err(|_| Error::device("ook_oregon: baro is not hex"))?;
            baro = (v + 856).to_string();
        }
        self.ss = nib[offset].ss as i64;
        self.es = nib[offset + 3].es as i64;
        self.putx(ctx, ANN_L2, vec![format!("{baro} mb"), baro]);
        Ok(())
    }

    /// pd.py `oregon_wind_dir()`.
    fn wind_dir(&mut self, ctx: &mut dyn Context, offset: usize) -> Result<()> {
        let nib = &self.decoded_nibbles;
        if !nib[offset].result.is_empty() {
            let v = i64::from_str_radix(&nib[offset].result, 16)
                .map_err(|_| Error::device("ook_oregon: nibble is not hex"))?;
            let w_dir = (v as f64 * 22.5) as i64;
            let idx = ((w_dir as f64 + 11.25) / 22.5).floor() as usize;
            let w_compass = DIR_TABLE.get(idx).copied().unwrap_or("N");
            self.ss = nib[offset].ss as i64;
            self.es = nib[offset].es as i64;
            self.putx(
                ctx,
                ANN_L2,
                vec![format!("{w_compass} ({w_dir}\u{00b0})"), w_compass.to_owned()],
            );
        }
        Ok(())
    }

    /// pd.py `oregon_channel()`.
    fn channel(&mut self, ctx: &mut dyn Context, offset: usize) -> Result<()> {
        let nib = &self.decoded_nibbles;
        let mut channel = String::new();
        if !nib[offset].result.is_empty() {
            let mut ch = i64::from_str_radix(&nib[offset].result, 16)
                .map_err(|_| Error::device("ook_oregon: nibble is not hex"))?;
            if self.ver != "v3" {
                if ch != 0 {
                    let mut bit_pos = 0;
                    while ch & 1 == 0 {
                        bit_pos += 1;
                        ch >>= 1;
                    }
                    if self.ver == "v2.1" {
                        bit_pos += 1;
                    }
                    channel = bit_pos.to_string();
                }
            } else if self.ver == "v3" {
                channel = ch.to_string();
            }
        }
        if !channel.is_empty() {
            self.ss = nib[offset].ss as i64;
            self.es = nib[offset].es as i64;
            self.putx(ctx, ANN_L2, vec![format!("Ch {channel}"), channel]);
        }
        Ok(())
    }

    /// pd.py `oregon_battery()`.
    fn battery(&mut self, ctx: &mut dyn Context, offset: usize) -> Result<()> {
        let nib = &self.decoded_nibbles;
        let mut batt = "OK";
        if !nib[offset].result.is_empty() {
            let v = i64::from_str_radix(&nib[offset].result, 16)
                .map_err(|_| Error::device("ook_oregon: nibble is not hex"))?;
            if (v >> 2) & 1 == 1 {
                batt = "Low";
            }
            self.ss = nib[offset].ss as i64;
            self.es = nib[offset].es as i64;
            self.putx(ctx, ANN_L2, vec![format!("Batt {batt}"), batt.to_owned()]);
        }
        Ok(())
    }

    /// pd.py `oregon_put_checksum()`.
    fn put_checksum(&mut self, ctx: &mut dyn Context, nibbles: usize, checksum: i64) -> Result<()> {
        let nib = &self.decoded_nibbles;
        let mut result = "BAD";
        if nibbles + 1 < nib.len() {
            if !nib[nibbles + 1].result.is_empty()
                && !nib[nibbles].result.is_empty()
                && checksum != -1
            {
                let next = i64::from_str_radix(&nib[nibbles + 1].result, 16)
                    .map_err(|_| Error::device("ook_oregon: checksum is not hex"))?;
                let cur = i64::from_str_radix(&nib[nibbles].result, 16)
                    .map_err(|_| Error::device("ook_oregon: checksum is not hex"))?;
                let explained = if self.ver != "v1" {
                    checksum == next * 16 + cur
                } else {
                    checksum == cur * 16 + next
                };
                if explained {
                    result = "OK";
                }
            }
            let rx_check =
                format!("{}{}", nib[nibbles + 1].result, nib[nibbles].result).to_uppercase();
            let details =
                format!("{result} Calc {} Rx {rx_check} ", hex_py(checksum).to_uppercase());
            self.ss = nib[nibbles].ss as i64;
            self.es = nib[nibbles + 1].es as i64;
            self.putx(ctx, ANN_L2, vec![format!("Checksum {details}"), result.to_owned()]);
        }
        Ok(())
    }

    /// pd.py `oregon_checksum()`.
    fn checksum(&mut self, ctx: &mut dyn Context, nibbles: usize) -> Result<()> {
        let mut checksum: i64 = 0;
        for i in 0..nibbles {
            let nibble = py_slice(&self.ookstring, i as i64 * 4, 4);
            let nibble: String = nibble.chars().rev().collect();
            if nibble.contains('E') {
                checksum = -1;
                break;
            }
            checksum += i64::from_str_radix(&nibble, 2)
                .map_err(|_| Error::device("ook_oregon: checksum nibble is not binary"))?;
            if checksum > 255 {
                checksum -= 255;
            }
        }
        if let Some((_, ver)) =
            SENSOR_CHECKSUM.iter().find(|(id, _)| *id == self.sensor_id.as_str())
        {
            self.ver = (*ver).to_owned();
        }
        if self.ver == "v2.1" {
            checksum -= 10;
        }
        self.put_checksum(ctx, nibbles, checksum)
    }

    /// pd.py `oregon_checksum_v1()`.
    fn checksum_v1(&mut self, ctx: &mut dyn Context) -> Result<()> {
        let nib = &self.decoded_nibbles;
        let mut checksum: i64 = 0;
        for i in 0..3 {
            if nib[2 * i].result.is_empty() || nib[2 * i + 1].result.is_empty() {
                checksum = -1;
                break;
            }
            let even = i64::from_str_radix(&nib[2 * i].result, 16)
                .map_err(|_| Error::device("ook_oregon: checksum is not hex"))?;
            let odd = i64::from_str_radix(&nib[2 * i + 1].result, 16)
                .map_err(|_| Error::device("ook_oregon: checksum is not hex"))?;
            checksum += ((even & 0xF) << 4) | (odd & 0xF);
            if checksum > 255 {
                checksum -= 255;
            }
        }
        self.put_checksum(ctx, 6, checksum)
    }

    /// pd.py `oregon_level2()`.
    fn level2(&mut self, ctx: &mut dyn Context) -> Result<()> {
        let nib = &self.decoded_nibbles;
        self.sensor_id =
            format!("{}{}{}{}", nib[0].result, nib[1].result, nib[2].result, nib[3].result)
                .to_uppercase();
        let (names_list, sensor_type) =
            match SENSORS.iter().find(|(id, _, _)| *id == self.sensor_id.as_str()) {
                Some((_, models, sensor_type)) => (*models, *sensor_type),
                None => (&["Unknown"][..], "Unknown"),
            };
        let names = names_list.join(",");
        let sensor_type = if sensor_type == "Unknown" && self.unknown != "Unknown" {
            self.unknown.clone()
        } else {
            sensor_type.to_owned()
        };
        self.ss = nib[0].ss as i64;
        self.es = nib[3].es as i64;
        self.putx(
            ctx,
            ANN_L2,
            vec![format!("{names} - {sensor_type}"), names.clone(), names_list[0].to_owned()],
        );
        self.channel(ctx, 4)?;
        self.battery(ctx, 7)?;
        match sensor_type.as_str() {
            "Rain" => {
                self.put_l2_param(ctx, 8, 4, 2, "", " in/hr")?;
                self.put_l2_param(ctx, 12, 6, 3, "Total ", " in")?;
                self.checksum(ctx, 18)?;
            }
            "Rain1" => {
                self.put_l2_param(ctx, 8, 3, 1, "", " mm/hr")?;
                self.put_l2_param(ctx, 11, 5, 1, "Total ", " mm")?;
                self.checksum(ctx, 18)?;
            }
            "Temp" => {
                self.temp(ctx, 8)?;
                self.checksum(ctx, 12)?;
            }
            "Temp_Hum_Baro" => {
                self.temp(ctx, 8)?;
                self.put_l2_param(ctx, 12, 2, 0, "Hum ", "%")?;
                self.baro(ctx, 15)?;
                self.checksum(ctx, 19)?;
            }
            "Temp_Hum_Baro1" => {
                self.temp(ctx, 8)?;
                self.put_l2_param(ctx, 12, 2, 0, "Hum ", "%")?;
                self.baro(ctx, 14)?;
            }
            "Temp_Hum" => {
                self.temp(ctx, 8)?;
                self.put_l2_param(ctx, 12, 2, 0, "Hum ", "%")?;
                self.checksum(ctx, 15)?;
            }
            "Temp_Hum1" => {
                self.temp(ctx, 8)?;
                self.put_l2_param(ctx, 12, 2, 0, "Hum ", "%")?;
                self.checksum(ctx, 14)?;
            }
            "UV" => {
                self.put_l2_param(ctx, 8, 2, 0, "", "")?;
            }
            "UV1" => {
                self.put_l2_param(ctx, 11, 2, 0, "", "")?;
            }
            "Wind" => {
                self.wind_dir(ctx, 8)?;
                self.put_l2_param(ctx, 11, 3, 1, "Gust ", " m/s")?;
                self.put_l2_param(ctx, 14, 3, 1, "Speed ", " m/s")?;
                self.checksum(ctx, 17)?;
            }
            _ => {}
        }
        Ok(())
    }

    /// pd.py `oregon_v1()`.
    fn v1(&mut self, ctx: &mut dyn Context) -> Result<()> {
        self.decode_pos = 0;
        self.decoded_nibbles.clear();
        if self.decoded.len() >= 32 {
            // Each call reads the current `decode_pos` for its span.
            let start = self.get_bit(self.decode_pos)?.ss;
            let end = self.get_bit(self.decode_pos + 3)?.es;
            self.put_nib(ctx, "RollingCode", start, end, 4)?;
            let start = self.get_bit(self.decode_pos)?.ss;
            let end = self.get_bit(self.decode_pos + 3)?.es;
            self.put_nib(ctx, "Ch", start, end, 4)?;
            let start = self.get_bit(self.decode_pos)?.ss;
            let end = self.get_bit(self.decode_pos + 15)?.es;
            self.put_nib(ctx, "Temp", start, end, 16)?;
            let start = self.get_bit(self.decode_pos)?.ss;
            let end = self.get_bit(self.decode_pos + 7)?.es;
            self.put_nib(ctx, "Checksum", start, end, 8)?;

            let last = self.decoded.len() - 1;
            self.dump_oregon_hex(ctx, self.decoded[0].ss, self.decoded[last].es);

            self.temp(ctx, 2)?;
            self.channel(ctx, 1)?;
            self.battery(ctx, 2)?;
            self.checksum_v1(ctx)?;
        }
        Ok(())
    }

    /// pd.py `oregon_v2()`: convert to the v3 format, discarding odd bits.
    fn v2(&mut self, ctx: &mut dyn Context) -> Result<()> {
        self.decode_pos = 0;
        self.ookstring = self.ookstring.chars().skip(1).step_by(2).collect();
        for i in 0..self.decoded.len() {
            if i % 2 == 1 {
                self.decoded[i].ss = self.decoded[i - 1].ss;
            }
        }
        self.decoded = self.decoded.iter().skip(1).step_by(2).copied().collect();
        self.v3(ctx)
    }

    /// pd.py `oregon_v3()`.
    fn v3(&mut self, ctx: &mut dyn Context) -> Result<()> {
        self.decode_pos = 0;
        self.decoded_nibbles.clear();
        if self.decoded.len() >= 32 {
            // The first span ends at the *start* of bit 16, not at the
            // end of bit 15.
            let start = self.get_bit(self.decode_pos)?.ss;
            let end = self.get_bit(self.decode_pos + 16)?.ss;
            self.put_nib(ctx, "SensorID", start, end, 16)?;
            let start = self.get_bit(self.decode_pos)?.ss;
            let end = self.get_bit(self.decode_pos + 3)?.es;
            self.put_nib(ctx, "Ch", start, end, 4)?;
            let start = self.get_bit(self.decode_pos)?.ss;
            let end = self.get_bit(self.decode_pos + 7)?.es;
            self.put_nib(ctx, "RollingCode", start, end, 8)?;
            let start = self.get_bit(self.decode_pos)?.ss;
            let end = self.get_bit(self.decode_pos + 3)?.es;
            self.put_nib(ctx, "Flags1", start, end, 4)?;

            let remaining = py_slice_from(&self.ookstring, self.decode_pos).len() / 4;
            for _ in 0..remaining {
                let start = self.get_bit(self.decode_pos)?.ss;
                let end = self.get_bit(self.decode_pos + 3)?.es;
                self.put_nib(ctx, "", start, end, 4)?;
            }
            let last = self.decoded.len() - 1;
            self.dump_oregon_hex(ctx, self.decoded[0].ss, self.decoded[last].es);
            self.level2(ctx)?;
        } else {
            let first = self.get_bit(0)?.ss;
            let last = self.get_bit(-1)?.es;
            self.ss = first as i64;
            self.es = last as i64;
            self.putx(ctx, ANN_FIELD, vec!["Too short to decode".to_owned()]);
        }
        Ok(())
    }

    /// pd.py `oregon()`.
    fn oregon(&mut self, ctx: &mut dyn Context) -> Result<()> {
        self.ookstring = self.decoded.iter().map(|b| b.state).collect();
        self.decode_pos = 0;

        enum Action {
            V21(usize),
            V1(usize),
            V3(usize),
            Error,
            None,
        }
        let action = {
            let s = &self.ookstring;
            let prefix = |n: usize| &s[..s.len().min(n)];
            if let Some(idx) = prefix(40).find("10011001") {
                if s.len() > idx + 8 && idx > 16 { Action::V21(idx) } else { Action::None }
            } else if let Some(idx) = prefix(17).find("E1100") {
                if s.len() > idx + 5 && idx <= 12 { Action::V1(idx) } else { Action::None }
            } else if let Some(idx) = prefix(28).find("0101") {
                if s.len() > idx + 4 && idx > 12 { Action::V3(idx) } else { Action::None }
            } else if s.len() > 16 {
                Action::Error
            } else {
                Action::None
            }
        };

        match action {
            Action::V21(idx) => {
                self.put_pre_and_sync(ctx, idx, 8, "v2.1")?;
                self.v2(ctx)?;
            }
            Action::V1(idx) => {
                self.put_pre_and_sync(ctx, idx, 5, "v1")?;
                self.v1(ctx)?;
            }
            Action::V3(idx) => {
                self.put_pre_and_sync(ctx, idx, 4, "v3")?;
                self.v3(ctx)?;
            }
            Action::Error => {
                self.ss = self.get_bit(0)?.ss as i64;
                self.es = self.get_bit(-1)?.es as i64;
                self.putx(ctx, ANN_FIELD, vec!["Not Oregon or wrong preamble".to_owned()]);
            }
            Action::None => {}
        }
        Ok(())
    }
}

impl Decoder for OokOregon {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, config: &Config) -> Result<()> {
        self.unknown = config.str("unknown", "Unknown").to_owned();
        Ok(())
    }

    fn decode(&mut self, _ctx: &mut dyn Context) -> Result<()> {
        Err(Error::not_supported("ook_oregon takes an ook decoder as input"))
    }

    fn decode_item(
        &mut self,
        _ss: u64,
        _es: u64,
        item: &Item,
        ctx: &mut dyn Context,
    ) -> Result<()> {
        let Item::List(bits) = item else {
            return Err(Error::device("ook_oregon: expected a list of OOK bits"));
        };
        let mut decoded = Vec::with_capacity(bits.len());
        for bit in bits {
            let Item::List(parts) = bit else {
                return Err(Error::device("ook_oregon: OOK bit is not a list"));
            };
            if parts.len() != 3 {
                return Err(Error::device("ook_oregon: OOK bit is not [ss, es, bit]"));
            }
            let (Item::Int(ss), Item::Int(es), Item::Str(state)) =
                (&parts[0], &parts[1], &parts[2])
            else {
                return Err(Error::device("ook_oregon: OOK bit fields are not [int, int, str]"));
            };
            decoded.push(OokBit {
                ss: *ss as u64,
                es: *es as u64,
                state: state.chars().next().unwrap_or('E'),
            });
        }
        self.decoded = decoded;
        self.oregon(ctx)
    }
}
