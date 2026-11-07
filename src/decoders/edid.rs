// Counterpart in sigrok:
//   libsigrokdecode/decoders/edid/pd.py,
//   Copyright (C) 2012 Bert Vermeulen <bert@biot.com>.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the I2C bus protocol and the decoder's documented classes and texts.

//! Extended Display Identification Data (EDID) 1.3 structure decoder.
//!
//! The decoder watches I²C address 0x50. A write selects the byte offset
//! (bit 7 selects an extension block), a read of 128 bytes then walks the
//! structure. It decodes the header, vendor/product/date, basic display
//! parameters, chromaticity, established and standard timings, the four
//! 18-byte descriptors and, for extension block 2, the CEA data block
//! collection and its DTDs.

use crate::core::{Error, Result};
use crate::decode::{AnnClass, AnnRow, Config, Context, Decoder, DecoderDef, DecoderInfo, Item};

const EDID_HEADER: [i64; 8] = [0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00];
const OFF_VENDOR: i64 = 8;
const OFF_VERSION: i64 = 18;
const OFF_BASIC: i64 = 20;
const OFF_CHROM: i64 = 25;
const OFF_EST_TIMING: i64 = 35;
const OFF_STD_TIMING: i64 = 38;
const OFF_DET_TIMING: i64 = 54;
const OFF_NUM_EXT: i64 = 126;
const OFF_CHECKSUM: i64 = 127;

/// Pre-EDID established timing modes.
const EST_MODES: [&str; 17] = [
    "720x400@70Hz",
    "720x400@88Hz",
    "640x480@60Hz",
    "640x480@67Hz",
    "640x480@72Hz",
    "640x480@75Hz",
    "800x600@56Hz",
    "800x600@60Hz",
    "800x600@72Hz",
    "800x600@75Hz",
    "832x624@75Hz",
    "1024x768@87Hz(i)",
    "1024x768@60Hz",
    "1024x768@70Hz",
    "1024x768@75Hz",
    "1280x1024@75Hz",
    "1152x870@75Hz",
];

/// X:Y display aspect ratios, as used in standard timing modes.
const XY_RATIO: [(i64, i64); 4] = [(16, 10), (4, 3), (5, 4), (16, 9)];

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "edid",
    name: "EDID",
    longname: "Extended Display Identification Data",
    desc: "Data structure describing display device capabilities.",
    license: "gplv3+",
    inputs: &["i2c"],
    outputs: &[],
    tags: &["Display", "Memory", "PC"],
    channels: &[],
    optional_channels: &[],
    options: &[],
    annotations: &[
        AnnClass { id: "fields", desc: "EDID structure fields" },
        AnnClass { id: "sections", desc: "EDID structure sections" },
    ],
    annotation_rows: &[
        AnnRow { id: "fields", desc: "Fields", classes: &[0] },
        AnnRow { id: "sections", desc: "Sections", classes: &[1] },
    ],
    binary: &[],
    // The doc is the package's __init__.py docstring, which the Python
    // host reads from the imported module.
    doc: "Extended Display Identification Data (EDID) 1.3 structure decoder.\n\
\n\
The three-character vendor ID as specified in the EDID standard refers to\n\
a Plug and Play ID (PNPID). The list of PNPID assignments is done by Microsoft.\n\
\n\
The 'pnpids.txt' file included with this protocol decoder is derived from\n\
the list of assignments downloadable from that page. It was retrieved in\n\
January 2012.\n\
\n\
Details:\n\
https://en.wikipedia.org/wiki/Extended_display_identification_data\n\
http://msdn.microsoft.com/en-us/windows/hardware/gg463195",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(Edid::default())
}

/// One data item's sample range.
#[derive(Clone, Copy)]
struct Sample {
    ss: u64,
    es: u64,
}

/// The state machine's states. Upstream starts with `None`.
#[derive(Clone, Copy, PartialEq, Eq)]
enum State {
    Offset,
    Header,
    Edid,
    Extensions,
}

/// The EDID decoder.
pub struct Edid {
    state: Option<State>,
    /// Received data items, used as an index into samplenum/data.
    cnt: i64,
    /// Start/end sample numbers per data item. `None` is upstream's
    /// integer `0` filler after a random offset write.
    sn: Vec<Option<Sample>>,
    /// Received data.
    cache: Vec<i64>,
    /// Random read offset.
    offset: i64,
    /// Extensions.
    extension: i64,
    ext_sn: Vec<Vec<Option<Sample>>>,
    ext_cache: Vec<Vec<i64>>,
    /// Set while decoding the basic display parameters.
    have_preferred_timing: bool,
    /// The sample number of the offset `ADDRESS WRITE`, when nonzero.
    ss: u64,
}

impl Default for Edid {
    fn default() -> Self {
        Edid {
            state: None,
            cnt: 0,
            sn: Vec::new(),
            cache: Vec::new(),
            offset: 0,
            extension: 0,
            ext_sn: vec![Vec::new()],
            ext_cache: vec![Vec::new()],
            have_preferred_timing: false,
            ss: 0,
        }
    }
}

/// An I2C packet is `[<ptype>, <pdata>]`.
fn split_packet(item: &Item) -> Result<(&str, &Item)> {
    let Item::List(parts) = item else {
        return Err(Error::device("edid: expected an I2C packet"));
    };
    if parts.len() != 2 {
        return Err(Error::device("edid: expected a two element I2C packet"));
    }
    let Item::Str(cmd) = &parts[0] else {
        return Err(Error::device("edid: packet type is not a string"));
    };
    Ok((cmd.as_str(), &parts[1]))
}

/// Python list indexing, negative indices from the end.
fn py_index(len: usize, i: i64) -> Result<usize> {
    let j = if i < 0 { i + len as i64 } else { i };
    if j < 0 || j >= len as i64 {
        return Err(Error::device("edid: index out of range"));
    }
    Ok(j as usize)
}

fn cache_at(cache: &[i64], i: i64) -> Result<i64> {
    Ok(cache[py_index(cache.len(), i)?])
}

fn sample_at(sn: &[Option<Sample>], i: i64) -> Result<Sample> {
    sn[py_index(sn.len(), i)?].ok_or_else(|| Error::device("edid: sample placeholder"))
}

/// CP437, as Python's `bytes.decode('cp437', errors='replace')`. EDID
/// text fields are often ASCII, but the standard does not require it.
fn cp437(data: &[i64]) -> String {
    const HIGH: [char; 128] = [
        'Ç', 'ü', 'é', 'â', 'ä', 'à', 'å', 'ç', 'ê', 'ë', 'è', 'ï', 'î', 'ì', 'Ä', 'Å', 'É', 'æ',
        'Æ', 'ô', 'ö', 'ò', 'û', 'ù', 'ÿ', 'Ö', 'Ü', '¢', '£', '¥', '₧', 'ƒ', 'á', 'í', 'ó', 'ú',
        'ñ', 'Ñ', 'ª', 'º', '¿', '⌐', '¬', '½', '¼', '¡', '«', '»', '░', '▒', '▓', '│', '┤', '╡',
        '╢', '╖', '╕', '╣', '║', '╗', '╝', '╜', '╛', '┐', '└', '┴', '┬', '├', '─', '┼', '╞', '╟',
        '╚', '╔', '╩', '╦', '╠', '═', '╬', '╧', '╨', '╤', '╥', '╙', '╘', '╒', '╓', '╫', '╪', '┘',
        '┌', '█', '▄', '▌', '▐', '▀', 'α', 'ß', 'Γ', 'π', 'Σ', 'σ', 'µ', 'τ', 'Φ', 'Θ', 'Ω', 'δ',
        '∞', 'φ', 'ε', '∩', '≡', '±', '≥', '≤', '⌠', '⌡', '÷', '≈', '°', '∙', '·', '√', 'ⁿ', '²',
        '■', '\u{a0}',
    ];
    data.iter()
        .map(|&b| {
            let b = (b & 0xff) as usize;
            if b < 0x80 { char::from_u32(b as u32).unwrap_or('\u{fffd}') } else { HIGH[b - 0x80] }
        })
        .collect()
}

impl Edid {
    /// As upstream's `ann_field`, choosing the extension cache when one
    /// is selected and using negative indices relative to its full list.
    fn ann_field(
        &self,
        ctx: &mut dyn Context,
        sn: &[Option<Sample>],
        start: i64,
        end: i64,
        texts: Vec<String>,
    ) -> Result<()> {
        let s = sample_at(sn, start)?;
        let e = sample_at(sn, end)?;
        ctx.put_ann(s.ss, e.es, 0, texts);
        Ok(())
    }

    /// The sample list `ann_field` uses.
    fn samples(&self) -> Result<&[Option<Sample>]> {
        if self.extension > 0 {
            let ext = usize::try_from(self.extension - 1)
                .map_err(|_| Error::device("edid: bad extension number"))?;
            Ok(self.ext_sn.get(ext).ok_or_else(|| Error::device("edid: extension not seen"))?)
        } else {
            Ok(&self.sn)
        }
    }

    fn cache_list(&self) -> Result<&[i64]> {
        if self.extension > 0 {
            let ext = usize::try_from(self.extension - 1)
                .map_err(|_| Error::device("edid: bad extension number"))?;
            Ok(self.ext_cache.get(ext).ok_or_else(|| Error::device("edid: extension not seen"))?)
        } else {
            Ok(&self.cache)
        }
    }

    /// Decode a video input identifier. Upstream reads `pnpids.txt`
    /// from the decoder package directory. That file is not part of the
    /// installed decoders tree (and this port ships no data files), so
    /// the vendor lookup finds nothing and is omitted.
    fn decode_vid(&mut self, ctx: &mut dyn Context, offset: i64) -> Result<()> {
        let b0 = cache_at(&self.cache, offset)?;
        let b1 = cache_at(&self.cache, offset + 1)?;
        let mut pnpid = String::new();
        pnpid.push(char::from_u32((64 + ((b0 & 0x7c) >> 2)) as u32).unwrap_or('?'));
        pnpid.push(
            char::from_u32((64 + (((b0 & 0x03) << 3) | ((b1 & 0xe0) >> 5))) as u32).unwrap_or('?'),
        );
        pnpid.push(char::from_u32((64 + (b1 & 0x1f)) as u32).unwrap_or('?'));
        self.ann_field(ctx, self.samples()?, offset, offset + 1, vec![pnpid])
    }

    fn decode_pid(&mut self, ctx: &mut dyn Context, offset: i64) -> Result<()> {
        let hi = cache_at(&self.cache, offset + 1)?;
        let lo = cache_at(&self.cache, offset)?;
        let text = format!("Product 0x{hi:02x}{lo:02x}");
        self.ann_field(ctx, self.samples()?, offset, offset + 1, vec![text])
    }

    fn decode_serial(&mut self, ctx: &mut dyn Context, offset: i64) -> Result<()> {
        let b0 = cache_at(&self.cache, offset)?;
        let b1 = cache_at(&self.cache, offset + 1)?;
        let b2 = cache_at(&self.cache, offset + 2)?;
        let b3 = cache_at(&self.cache, offset + 3)?;
        let serialnum = (b3 << 24) + (b2 << 16) + (b1 << 8) + b0;
        let mut serialstr = String::new();
        let mut is_alnum = true;
        for i in 0..4 {
            let b = cache_at(&self.cache, offset + 3 - i)?;
            let ch = char::from_u32((b & 0xff) as u32).unwrap_or('\u{fffd}');
            if !ch.is_alphanumeric() {
                is_alnum = false;
                break;
            }
            serialstr.push(ch);
        }
        let serial = if is_alnum { serialstr } else { serialnum.to_string() };
        self.ann_field(ctx, self.samples()?, offset, offset + 3, vec![format!("Serial {serial}")])
    }

    fn decode_mfrdate(&mut self, ctx: &mut dyn Context, offset: i64) -> Result<()> {
        let week = cache_at(&self.cache, offset)?;
        let year = cache_at(&self.cache, offset + 1)?;
        let mut datestr = String::new();
        if week != 0 {
            datestr.push_str(&format!("week {week}, "));
        }
        datestr.push_str(&(1990 + year).to_string());
        self.ann_field(
            ctx,
            self.samples()?,
            offset,
            offset + 1,
            vec![format!("Manufactured {datestr}"), datestr],
        )
    }

    fn decode_basicdisplay(&mut self, ctx: &mut dyn Context, offset: i64) -> Result<()> {
        let sn = self.samples()?.to_vec();
        let cache = self.cache.clone();
        // Video input definition.
        let vid = cache_at(&cache, offset)?;
        if vid & 0x80 != 0 {
            // Digital.
            self.ann_field(ctx, &sn, offset, offset, vec!["Video input: VESA DFP 1.".to_owned()])?;
        } else {
            // Analog.
            let sls = (vid & 60) >> 5;
            self.ann_field(
                ctx,
                &sn,
                offset,
                offset,
                vec![format!("Signal level standard: {sls:02x}")],
            )?;
            if vid & 0x10 != 0 {
                self.ann_field(
                    ctx,
                    &sn,
                    offset,
                    offset,
                    vec!["Blank-to-black setup expected".to_owned()],
                )?;
            }
            let mut syncs = String::new();
            if vid & 0x08 != 0 {
                syncs.push_str("separate syncs, ");
            }
            if vid & 0x04 != 0 {
                syncs.push_str("composite syncs, ");
            }
            if vid & 0x02 != 0 {
                syncs.push_str("sync on green, ");
            }
            if vid & 0x01 != 0 {
                syncs.push_str("Vsync serration required, ");
            }
            if !syncs.is_empty() {
                let text = format!("Supported syncs: {}", &syncs[..syncs.len() - 2]);
                self.ann_field(ctx, &sn, offset, offset, vec![text])?;
            }
        }
        // Max horizontal/vertical image size. Projectors have this set to 0.
        let hsize = cache_at(&cache, offset + 1)?;
        let vsize = cache_at(&cache, offset + 2)?;
        if hsize != 0 && vsize != 0 {
            let text = format!("Physical size: {hsize}x{vsize}cm");
            self.ann_field(ctx, &sn, offset + 1, offset + 2, vec![text])?;
        }
        // Display transfer characteristic (gamma).
        let gamma_byte = cache_at(&cache, offset + 3)?;
        if gamma_byte != 0xff {
            let gamma = (gamma_byte + 100) as f64 / 100.0;
            self.ann_field(ctx, &sn, offset + 3, offset + 3, vec![format!("Gamma: {gamma:.2}")])?;
        }
        // Feature support.
        let fs = cache_at(&cache, offset + 4)?;
        let mut dpms = String::new();
        if fs & 0x80 != 0 {
            dpms.push_str("standby, ");
        }
        if fs & 0x40 != 0 {
            dpms.push_str("suspend, ");
        }
        if fs & 0x20 != 0 {
            dpms.push_str("active off, ");
        }
        if !dpms.is_empty() {
            let text = format!("DPMS support: {}", &dpms[..dpms.len() - 2]);
            self.ann_field(ctx, &sn, offset + 4, offset + 4, vec![text])?;
        }
        let dt = (fs & 0x18) >> 3;
        let dtstr = match dt {
            0 => "Monochrome",
            1 => "RGB color",
            2 => "non-RGB multicolor",
            _ => "",
        };
        if !dtstr.is_empty() {
            self.ann_field(
                ctx,
                &sn,
                offset + 4,
                offset + 4,
                vec![format!("Display type: {dtstr}")],
            )?;
        }
        if fs & 0x04 != 0 {
            self.ann_field(
                ctx,
                &sn,
                offset + 4,
                offset + 4,
                vec!["Color space: standard sRGB".to_owned()],
            )?;
        }
        // Save this for when we decode the first detailed timing descriptor.
        self.have_preferred_timing = (fs & 0x02) == 0x02;
        let gft = if fs & 0x01 != 0 { "" } else { "not " };
        self.ann_field(
            ctx,
            &sn,
            offset + 4,
            offset + 4,
            vec![format!("Generalized timing formula: {gft}supported")],
        )
    }

    /// Convert from 10-bit packet format to float.
    fn convert_color(mut value: i64) -> f64 {
        let mut outval = 0.0;
        for i in 0..10 {
            if value & 0x01 != 0 {
                outval += 2f64.powi(-(10 - i));
            }
            value >>= 1;
        }
        outval
    }

    fn decode_chromaticity(&mut self, ctx: &mut dyn Context, offset: i64) -> Result<()> {
        let cache = self.cache.clone();
        let sn = self.samples()?.to_vec();
        let c = |i: i64| cache_at(&cache, offset + i);

        let redx = (c(2)? << 2) + ((c(0)? & 0xc0) >> 6);
        let redy = (c(3)? << 2) + ((c(0)? & 0x30) >> 4);
        self.ann_field(
            ctx,
            &sn,
            offset,
            offset + 9,
            vec![format!(
                "Chromacity red: X {:.3}, Y {:.3}",
                Self::convert_color(redx),
                Self::convert_color(redy)
            )],
        )?;

        // Upstream shifts these two masks by 6 and 4, so they are always
        // zero; reproduced rather than corrected.
        let greenx = (c(4)? << 2) + ((c(0)? & 0x0c) >> 6);
        let greeny = (c(5)? << 2) + ((c(0)? & 0x03) >> 4);
        self.ann_field(
            ctx,
            &sn,
            offset,
            offset + 9,
            vec![format!(
                "Chromacity green: X {:.3}, Y {:.3}",
                Self::convert_color(greenx),
                Self::convert_color(greeny)
            )],
        )?;

        let bluex = (c(6)? << 2) + ((c(1)? & 0xc0) >> 6);
        let bluey = (c(7)? << 2) + ((c(1)? & 0x30) >> 4);
        self.ann_field(
            ctx,
            &sn,
            offset,
            offset + 9,
            vec![format!(
                "Chromacity blue: X {:.3}, Y {:.3}",
                Self::convert_color(bluex),
                Self::convert_color(bluey)
            )],
        )?;

        let whitex = (c(8)? << 2) + ((c(1)? & 0x0c) >> 6);
        let whitey = (c(9)? << 2) + ((c(1)? & 0x03) >> 4);
        self.ann_field(
            ctx,
            &sn,
            offset,
            offset + 9,
            vec![format!(
                "Chromacity white: X {:.3}, Y {:.3}",
                Self::convert_color(whitex),
                Self::convert_color(whitey)
            )],
        )
    }

    fn decode_est_timing(&mut self, ctx: &mut dyn Context, offset: i64) -> Result<()> {
        // Pre-EDID modes.
        let bitmap = (cache_at(&self.cache, offset)? << 9)
            + (cache_at(&self.cache, offset + 1)? << 1)
            + ((cache_at(&self.cache, offset + 2)? & 0x80) >> 7);
        let mut modestr = String::new();
        for i in 0..17 {
            if bitmap & (1 << (16 - i)) != 0 {
                modestr.push_str(EST_MODES[i as usize]);
                modestr.push_str(", ");
            }
        }
        if !modestr.is_empty() {
            let text = format!("Supported established modes: {}", &modestr[..modestr.len() - 2]);
            self.ann_field(ctx, self.samples()?, offset, offset + 2, vec![text])?;
        }
        Ok(())
    }

    fn decode_std_timing(&mut self, ctx: &mut dyn Context, offset: i64) -> Result<()> {
        let mut modestr = String::new();
        for i in (0..16).step_by(2) {
            let lo = cache_at(&self.cache, offset + i)?;
            let hi = cache_at(&self.cache, offset + i + 1)?;
            if lo == 0x01 && hi == 0x01 {
                // Unused field.
                continue;
            }
            let x = (lo + 31) * 8;
            let ratio = ((hi & 0xc0) >> 6) as usize;
            let (ratio_x, ratio_y) = XY_RATIO[ratio];
            // Python 3 `/` is float; `%d` truncates toward zero.
            let y = (x as f64 / ratio_x as f64 * ratio_y as f64) as i64;
            let refresh = (hi & 0x3f) + 60;
            modestr.push_str(&format!("{x}x{y}@{refresh}Hz, "));
        }
        if !modestr.is_empty() {
            let text = format!("Supported standard modes: {}", &modestr[..modestr.len() - 2]);
            self.ann_field(ctx, self.samples()?, offset, offset + 15, vec![text])?;
        }
        Ok(())
    }

    fn decode_detailed_timing(
        &self,
        ctx: &mut dyn Context,
        cache: &[i64],
        sn: &[Option<Sample>],
        full_sn: &[Option<Sample>],
        offset: i64,
        is_first: bool,
    ) -> Result<()> {
        let section = if is_first && self.have_preferred_timing {
            // Only on first detailed timing descriptor.
            "Preferred timing descriptor"
        } else {
            "Detailed timing descriptor"
        };
        let first = sample_at(sn, 0)?;
        let last = sample_at(sn, 17)?;
        ctx.put_ann(first.ss, last.es, 1, vec![section.to_owned()]);

        let pixclock = ((cache[1] << 8) + cache[0]) as f64 / 100.0;
        self.ann_field(
            ctx,
            full_sn,
            offset,
            offset + 1,
            vec![format!("Pixel clock: {pixclock:.2} MHz")],
        )?;

        let horiz_active = ((cache[4] & 0xf0) << 4) + cache[2];
        let horiz_blank = ((cache[4] & 0x0f) << 8) + cache[3];
        self.ann_field(
            ctx,
            full_sn,
            offset + 2,
            offset + 4,
            vec![format!("Horizontal active: {horiz_active}, blanking: {horiz_blank}")],
        )?;

        let vert_active = ((cache[7] & 0xf0) << 4) + cache[5];
        let vert_blank = ((cache[7] & 0x0f) << 8) + cache[6];
        self.ann_field(
            ctx,
            full_sn,
            offset + 5,
            offset + 7,
            vec![format!("Vertical active: {vert_active}, blanking: {vert_blank}")],
        )?;

        let horiz_sync_off = ((cache[11] & 0xc0) << 2) + cache[8];
        let horiz_sync_pw = ((cache[11] & 0x30) << 4) + cache[9];
        let vert_sync_off = ((cache[11] & 0x0c) << 2) + ((cache[10] & 0xf0) >> 4);
        let vert_sync_pw = ((cache[11] & 0x03) << 4) + (cache[10] & 0x0f);
        self.ann_field(
            ctx,
            full_sn,
            offset + 8,
            offset + 11,
            vec![
                format!(
                    "Horizontal sync offset: {horiz_sync_off}, pulse width: {horiz_sync_pw}, Vertical sync offset: {vert_sync_off}, pulse width: {vert_sync_pw}"
                ),
                format!(
                    "HSync off: {horiz_sync_off}, pw: {horiz_sync_pw}, VSync off: {vert_sync_off}, pw: {vert_sync_pw}"
                ),
            ],
        )?;

        let horiz_size = ((cache[14] & 0xf0) << 4) + cache[12];
        let vert_size = ((cache[14] & 0x0f) << 8) + cache[13];
        self.ann_field(
            ctx,
            full_sn,
            offset + 12,
            offset + 14,
            vec![format!("Physical size: {horiz_size}x{vert_size}mm")],
        )?;

        let horiz_border = cache[15];
        self.ann_field(
            ctx,
            full_sn,
            offset + 15,
            offset + 15,
            vec![format!("Horizontal border: {horiz_border} pixels")],
        )?;
        let vert_border = cache[16];
        self.ann_field(
            ctx,
            full_sn,
            offset + 16,
            offset + 16,
            vec![format!("Vertical border: {vert_border} lines")],
        )?;

        let mut features = String::from("Flags: ");
        if cache[17] & 0x80 != 0 {
            features.push_str("interlaced, ");
        }
        let stereo = (cache[17] & 0x60) >> 5;
        if stereo != 0 {
            if cache[17] & 0x01 != 0 {
                features.push_str("2-way interleaved stereo (");
                features.push_str(
                    ["right image on even lines", "left image on even lines", "side-by-side"]
                        [(stereo - 1) as usize],
                );
                features.push_str("), ");
            } else {
                features.push_str("field sequential stereo (");
                features.push_str(
                    ["right image on sync=1", "left image on sync=1", "4-way interleaved"]
                        [(stereo - 1) as usize],
                );
                features.push_str("), ");
            }
        }
        let sync = (cache[17] & 0x18) >> 3;
        let sync2 = (cache[17] & 0x06) >> 1;
        let posneg = ["negative", "positive"];
        features.push_str("sync type ");
        match sync {
            0x00 => features.push_str("analog composite (serrate on RGB)"),
            0x01 => features.push_str("bipolar analog composite (serrate on RGB)"),
            0x02 => {
                features.push_str("digital composite (serrate on composite polarity ");
                features.push_str(posneg[(sync2 & 0x01) as usize]);
                features.push(')');
            }
            0x03 => {
                features.push_str("digital separate (");
                features.push_str("Vsync polarity ");
                features.push_str(posneg[((sync2 & 0x02) >> 1) as usize]);
                features.push_str(", Hsync polarity ");
                features.push_str(posneg[(sync2 & 0x01) as usize]);
                features.push(')');
            }
            _ => {}
        }
        features.push_str(", ");
        self.ann_field(
            ctx,
            full_sn,
            offset + 17,
            offset + 17,
            vec![features[..features.len() - 2].to_owned()],
        )
    }

    fn decode_descriptor(
        &self,
        ctx: &mut dyn Context,
        cache: &[i64],
        full_sn: &[Option<Sample>],
        offset: i64,
    ) -> Result<()> {
        let tag = cache[3];
        self.ann_field(ctx, full_sn, offset, offset + 1, vec!["Flag".to_owned()])?;
        self.ann_field(ctx, full_sn, offset + 2, offset + 2, vec!["Flag (reserved)".to_owned()])?;
        self.ann_field(ctx, full_sn, offset + 3, offset + 3, vec![format!("Tag: {tag:X}")])?;
        self.ann_field(ctx, full_sn, offset + 4, offset + 4, vec!["Flag".to_owned()])?;

        let first = sample_at(full_sn, offset)?;
        let last = sample_at(full_sn, offset + 17)?;
        let text = |data: &[i64]| -> String {
            let mut bytes = Vec::new();
            for &b in data {
                bytes.push(b);
            }
            cp437(&bytes).trim_matches(char::is_whitespace).to_owned()
        };
        let field_end = (5 + 13).min(cache.len());
        let field_start = 5.min(field_end);
        let field = &cache[field_start..field_end];
        match tag {
            0xff => {
                // Monitor serial number.
                ctx.put_ann(first.ss, last.es, 1, vec!["Serial number".to_owned()]);
                let value = text(field);
                self.ann_field(ctx, full_sn, offset + 5, offset + 17, vec![value])?;
            }
            0xfe => {
                // Text.
                ctx.put_ann(first.ss, last.es, 1, vec!["Text".to_owned()]);
                let value = text(field);
                self.ann_field(ctx, full_sn, offset + 5, offset + 17, vec![value])?;
            }
            0xfc => {
                // Monitor name.
                ctx.put_ann(first.ss, last.es, 1, vec!["Monitor name".to_owned()]);
                let value = text(field);
                self.ann_field(ctx, full_sn, offset + 5, offset + 17, vec![value])?;
            }
            0xfd => {
                // Monitor range limits.
                ctx.put_ann(first.ss, last.es, 1, vec!["Monitor range limits".to_owned()]);
                let min_v = cache[5];
                self.ann_field(
                    ctx,
                    full_sn,
                    offset + 5,
                    offset + 5,
                    vec![
                        format!("Minimum vertical rate: {min_v}Hz"),
                        format!("VSync >= {min_v}Hz"),
                    ],
                )?;
                let max_v = cache[6];
                self.ann_field(
                    ctx,
                    full_sn,
                    offset + 6,
                    offset + 6,
                    vec![
                        format!("Maximum vertical rate: {max_v}Hz"),
                        format!("VSync <= {max_v}Hz"),
                    ],
                )?;
                let min_h = cache[7];
                self.ann_field(
                    ctx,
                    full_sn,
                    offset + 7,
                    offset + 7,
                    vec![
                        format!("Minimum horizontal rate: {min_h}kHz"),
                        format!("HSync >= {min_h}kHz"),
                    ],
                )?;
                let max_h = cache[8];
                self.ann_field(
                    ctx,
                    full_sn,
                    offset + 8,
                    offset + 8,
                    vec![
                        format!("Maximum horizontal rate: {max_h}kHz"),
                        format!("HSync <= {max_h}kHz"),
                    ],
                )?;
                let pixclk = cache[9] * 10;
                self.ann_field(
                    ctx,
                    full_sn,
                    offset + 9,
                    offset + 9,
                    vec![
                        format!("Maximum pixel clock: {pixclk}MHz"),
                        format!("PixClk <= {pixclk}MHz"),
                    ],
                )?;
                if cache[10] == 0x02 {
                    self.ann_field(
                        ctx,
                        full_sn,
                        offset + 10,
                        offset + 10,
                        vec![
                            "Secondary timing formula supported".to_owned(),
                            "2nd GTF: yes".to_owned(),
                        ],
                    )?;
                    self.ann_field(ctx, full_sn, offset + 11, offset + 17, vec!["GTF".to_owned()])?;
                } else {
                    self.ann_field(
                        ctx,
                        full_sn,
                        offset + 10,
                        offset + 10,
                        vec![
                            "Secondary timing formula unsupported".to_owned(),
                            "2nd GTF: no".to_owned(),
                        ],
                    )?;
                    self.ann_field(
                        ctx,
                        full_sn,
                        offset + 11,
                        offset + 17,
                        vec!["Padding".to_owned()],
                    )?;
                }
            }
            0xfb => {
                // Additional color point data.
                ctx.put_ann(first.ss, last.es, 1, vec!["Additional color point data".to_owned()]);
            }
            0xfa => {
                // Additional standard timing definitions.
                ctx.put_ann(
                    first.ss,
                    last.es,
                    1,
                    vec!["Additional standard timing definitions".to_owned()],
                );
            }
            _ => {
                ctx.put_ann(first.ss, last.es, 1, vec!["Unknown descriptor".to_owned()]);
            }
        }
        Ok(())
    }

    fn decode_descriptors(&mut self, ctx: &mut dyn Context, offset: i64) -> Result<()> {
        // 4 consecutive 18-byte descriptor blocks.
        let cache = self.cache_list()?.to_vec();
        let sn = self.samples()?.to_vec();

        let mut i = offset;
        while i < 0 {
            let start = py_index(cache.len(), i)?;
            if start + 1 < cache.len() && (cache[start] != 0 || cache[start + 1] != 0) {
                let end = (start + 18).min(cache.len());
                self.decode_detailed_timing(
                    ctx,
                    &cache[start..end],
                    &sn[start..end],
                    &sn,
                    i,
                    i == offset,
                )?;
            } else if cache[start + 2] == 0 || cache[start + 4] == 0 {
                self.decode_descriptor(ctx, &cache[start..], &sn, i)?;
            }
            i += 18;
        }
        Ok(())
    }

    fn decode_data_block(
        &self,
        ctx: &mut dyn Context,
        tag: i64,
        cache: &[i64],
        sn: &[Option<Sample>],
    ) -> Result<()> {
        let codes = |t: i64| -> Vec<String> {
            match t {
                0 => vec!["0: Reserved".to_owned()],
                1 => vec!["1: Audio Data Block".to_owned(), "Audio".to_owned()],
                2 => vec!["2: Video Data Block".to_owned(), "Video".to_owned()],
                3 => vec!["3: Vendor Specific Data Block".to_owned(), "VSDB".to_owned()],
                4 => vec!["4: Speacker Allocation Data Block".to_owned(), "SADB".to_owned()],
                5 => vec!["5: VESA DTC Data Block".to_owned(), "DTC".to_owned()],
                6 => vec!["6: Reserved".to_owned()],
                _ => vec!["7: Extended".to_owned(), "Ext".to_owned()],
            }
        };
        let ext_codes = |t: i64| -> [&'static str; 2] {
            match t {
                0 => ["0: Video Capability Data Block", "VCDB"],
                1 => ["1: Vendor Specific Video Data Block", "VSVDB"],
                17 => ["17: Vendor Specific Audio Data Block", "VSADB"],
                _ => ["Unknown", "?"],
            }
        };

        let (code, data, ext_len) = if tag < 7 {
            let code = codes(tag);
            let data = match tag {
                1 => {
                    // Audio data block.
                    let aformats = |a: i64| -> String {
                        if a == 1 { "1 (LPCM)".to_owned() } else { a.to_string() }
                    };
                    let rates = ["192", "176", "96", "88", "48", "44", "32"];
                    let aformat = cache[1] >> 3;
                    let mut sup_rates = Vec::new();
                    for i in 0..8i64 {
                        if (1 << i) & cache[2] != 0 {
                            sup_rates.push(i);
                        }
                    }
                    let mut data =
                        format!("Format: {} Channels: {}", aformats(aformat), (cache[1] & 0x7) + 1);
                    let mut joined = Vec::new();
                    for i in sup_rates {
                        let idx = if i <= 6 { (6 - i) as usize } else { 6 };
                        joined.push(rates[idx].to_owned());
                    }
                    data.push_str(" Rates: ");
                    data.push_str(&joined.join(" "));
                    data.push_str(&format!(" Extra: [{:02X}]", cache[3]));
                    data
                }
                2 => {
                    let mut items = Vec::new();
                    for &v in &cache[1..] {
                        items.push(format!(
                            "{}{}",
                            v & 0x7f,
                            if v >> 7 != 0 { " (Native)" } else { "" }
                        ));
                    }
                    format!("VIC: {}", items.join(", "))
                }
                3 => {
                    // Vendor specific data block. Python's cache[3:0:-1]
                    // is indices 3, 2, 1 and truncates when short.
                    let mut oui = Vec::new();
                    for idx in [3, 2, 1] {
                        if idx < cache.len() {
                            oui.push(cache[idx]);
                        }
                    }
                    let is_hdmi = oui == [0x00, 0x0c, 0x03];
                    let mut data = format!(
                        "OUI: {}",
                        oui.iter().map(|x| format!("{x:02X}")).collect::<Vec<_>>().join(" ")
                    );
                    if is_hdmi {
                        data.push_str(" (HDMI Licensing, LLC)");
                    }
                    data.push_str(&format!(
                        ", PhyAddr: {}.{}.{}.{}",
                        cache[4] >> 4,
                        cache[4] & 0xf,
                        cache[5] >> 4,
                        cache[5] & 0xf
                    ));
                    data.push_str(", [");
                    data.push_str(
                        &cache[6..]
                            .iter()
                            .map(|x| format!("{x:02X}"))
                            .collect::<Vec<_>>()
                            .join(" "),
                    );
                    data.push(']');
                    data
                }
                4 => {
                    let speakers = [
                        "FL/FR", "LFE", "FC", "RL/RR", "RC", "FLC/FRC", "RLC/RRC", "FLW/FRW",
                        "FLH/FRH", "TC", "FCH",
                    ];
                    let sup_speakers = cache[1] + (cache[2] << 8);
                    let mut items = Vec::new();
                    for i in 0..8i64 {
                        if (1 << i) & sup_speakers != 0 {
                            items.push(speakers[i as usize]);
                        }
                    }
                    format!("Speakers: {}", items.join(" "))
                }
                _ => {
                    let items: Vec<String> =
                        cache[1..].iter().map(|x| format!("{x:02X}")).collect();
                    items.join(" ")
                }
            };
            (code, data, 0)
        } else {
            // Extended tags.
            let ext = ext_codes(cache[1]);
            let base = codes(7);
            let code = vec![
                format!("{}{}{}", base[0], ", ", ext[0]),
                format!("{}{}{}", base[1], ": ", ext[1]),
            ];
            let items: Vec<String> = cache[2..].iter().map(|x| format!("{x:02X}")).collect();
            (code, items.join(" "), 1)
        };

        let first = sample_at(sn, 0)?;
        let code_end = sample_at(sn, ext_len)?;
        ctx.put_ann(first.ss, code_end.es, 0, code);
        let data_start = sample_at(sn, 1 + ext_len)?;
        let data_end = sample_at(sn, cache.len() as i64 - 1)?;
        ctx.put_ann(data_start.ss, data_end.es, 0, vec![data]);
        Ok(())
    }

    fn decode_data_block_collection(
        &mut self,
        ctx: &mut dyn Context,
        cache: &[i64],
        sn: &[Option<Sample>],
    ) -> Result<()> {
        let mut offset = 0usize;
        while offset < cache.len() {
            let length = ((1 + cache[offset]) & 0x1f) as usize;
            let tag = cache[offset] >> 5;
            let end = (offset + length).min(cache.len());
            self.decode_data_block(ctx, tag, &cache[offset..end], &sn[offset..end])?;
            offset += length;
        }
        Ok(())
    }
}

impl Decoder for Edid {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, _config: &Config) -> Result<()> {
        *self = Edid::default();
        Ok(())
    }

    fn decode(&mut self, _ctx: &mut dyn Context) -> Result<()> {
        Err(Error::not_supported("edid takes an i2c decoder as input"))
    }

    fn decode_item(&mut self, ss: u64, es: u64, item: &Item, ctx: &mut dyn Context) -> Result<()> {
        let (cmd, data) = split_packet(item)?;
        let databyte = match data {
            Item::Int(v) => Some(*v),
            _ => None,
        };

        if cmd == "ADDRESS WRITE" && databyte == Some(0x50) {
            self.state = Some(State::Offset);
            self.ss = ss;
            return Ok(());
        }

        if cmd == "ADDRESS READ" && databyte == Some(0x50) {
            let texts = if self.extension > 0 {
                self.state = Some(State::Extensions);
                let s = self.extension.to_string();
                vec![format!("Extension: {s}"), format!("X: {s}"), s]
            } else {
                self.state = Some(State::Header);
                vec!["EDID".to_owned()]
            };
            ctx.put_ann(ss, es, 1, texts);
            return Ok(());
        }

        if cmd == "DATA WRITE" && self.state == Some(State::Offset) {
            let Some(offset) = databyte else {
                return Err(Error::device("edid: offset write has no data byte"));
            };
            self.offset = offset;
            // Python's `//` and `%`; the offset is a byte.
            self.extension = offset / 128;
            self.cnt = offset % 128;
            if self.extension > 0 {
                let ext = usize::try_from(self.extension - 1)
                    .map_err(|_| Error::device("edid: bad extension number"))?;
                let ext_sn = self
                    .ext_sn
                    .get(ext)
                    .ok_or_else(|| Error::device("edid: extension not seen"))?
                    .clone();
                let ext_cache = self
                    .ext_cache
                    .get(ext)
                    .ok_or_else(|| Error::device("edid: extension not seen"))?
                    .clone();
                self.sn = resize_sn(ext_sn, self.cnt);
                self.cache = resize_cache(ext_cache, self.cnt);
            } else {
                self.sn = resize_sn(self.sn.clone(), self.cnt);
                self.cache = resize_cache(self.cache.clone(), self.cnt);
            }
            let ss = if self.ss != 0 { self.ss } else { ss };
            let s = offset.to_string();
            ctx.put_ann(ss, es, 1, vec![format!("Offset: {s}"), format!("O: {s}"), s]);
            return Ok(());
        }

        // We only care about actual data bytes that are read (for now).
        if cmd != "DATA READ" {
            return Ok(());
        }
        let Some(data) = databyte else {
            return Err(Error::device("edid: DATA READ has no data byte"));
        };

        self.cnt += 1;
        if self.extension > 0 {
            let ext = usize::try_from(self.extension - 1)
                .map_err(|_| Error::device("edid: bad extension number"))?;
            let ext_sn = self
                .ext_sn
                .get_mut(ext)
                .ok_or_else(|| Error::device("edid: extension not seen"))?;
            ext_sn.push(Some(Sample { ss, es }));
            let ext_cache = self
                .ext_cache
                .get_mut(ext)
                .ok_or_else(|| Error::device("edid: extension not seen"))?;
            ext_cache.push(data);
        } else {
            self.sn.push(Some(Sample { ss, es }));
            self.cache.push(data);
        }

        match self.state {
            None | Some(State::Header) => {
                // Wait for the EDID header.
                if self.cnt >= OFF_VENDOR {
                    let len = self.cache.len();
                    let start = len.saturating_sub(8);
                    if self.cache[start..] == EDID_HEADER {
                        // Throw away any garbage before the header.
                        let keep = self.sn.len().saturating_sub(8);
                        self.sn = self.sn[keep..].to_vec();
                        let keepc = self.cache.len().saturating_sub(8);
                        self.cache = self.cache[keepc..].to_vec();
                        self.cnt = 8;
                        self.state = Some(State::Edid);
                        let first = sample_at(&self.sn, 0)?;
                        ctx.put_ann(first.ss, es, 1, vec!["Header".to_owned()]);
                        ctx.put_ann(first.ss, es, 0, vec!["Header pattern".to_owned()]);
                    }
                }
            }
            Some(State::Edid) => {
                self.decode_edid_byte(ctx, ss, es)?;
            }
            Some(State::Extensions) => {
                self.decode_extension_byte(ctx, ss, es)?;
            }
            Some(State::Offset) => {}
        }
        Ok(())
    }
}

/// Resize a sample list to `cnt` entries, padding with `None` as
/// upstream pads with the integer 0.
fn resize_sn(mut v: Vec<Option<Sample>>, cnt: i64) -> Vec<Option<Sample>> {
    let cnt = usize::try_from(cnt).unwrap_or(0);
    v.truncate(cnt);
    v.resize(cnt, None);
    v
}

fn resize_cache(mut v: Vec<i64>, cnt: i64) -> Vec<i64> {
    let cnt = usize::try_from(cnt).unwrap_or(0);
    v.truncate(cnt);
    v.resize(cnt, 0);
    v
}

impl Edid {
    fn decode_edid_byte(&mut self, ctx: &mut dyn Context, ss: u64, es: u64) -> Result<()> {
        let cnt = self.cnt;
        if cnt == OFF_VERSION {
            self.decode_vid(ctx, -10)?;
            self.decode_pid(ctx, -8)?;
            self.decode_serial(ctx, -6)?;
            self.decode_mfrdate(ctx, -2)?;
            let sn = self.samples()?.to_vec();
            ctx.put_ann(sample_at(&sn, OFF_VENDOR)?.ss, es, 1, vec!["Vendor/product".to_owned()]);
        } else if cnt == OFF_BASIC {
            let sn = self.samples()?.to_vec();
            ctx.put_ann(sample_at(&sn, OFF_VERSION)?.ss, es, 1, vec!["EDID Version".to_owned()]);
            let s = sample_at(&sn, OFF_VERSION)?;
            ctx.put_ann(s.ss, s.es, 0, vec![format!("Version {}", cache_at(&self.cache, -2)?)]);
            let s = sample_at(&sn, OFF_VERSION + 1)?;
            ctx.put_ann(s.ss, s.es, 0, vec![format!("Revision {}", cache_at(&self.cache, -1)?)]);
        } else if cnt == OFF_CHROM {
            let sn = self.samples()?.to_vec();
            ctx.put_ann(sample_at(&sn, OFF_BASIC)?.ss, es, 1, vec!["Basic display".to_owned()]);
            self.decode_basicdisplay(ctx, -5)?;
        } else if cnt == OFF_EST_TIMING {
            let sn = self.samples()?.to_vec();
            ctx.put_ann(
                sample_at(&sn, OFF_CHROM)?.ss,
                es,
                1,
                vec!["Color characteristics".to_owned()],
            );
            self.decode_chromaticity(ctx, -10)?;
        } else if cnt == OFF_STD_TIMING {
            let sn = self.samples()?.to_vec();
            ctx.put_ann(
                sample_at(&sn, OFF_EST_TIMING)?.ss,
                es,
                1,
                vec!["Established timings".to_owned()],
            );
            self.decode_est_timing(ctx, -3)?;
        } else if cnt == OFF_DET_TIMING {
            let sn = self.samples()?.to_vec();
            ctx.put_ann(
                sample_at(&sn, OFF_STD_TIMING)?.ss,
                es,
                1,
                vec!["Standard timings".to_owned()],
            );
            self.decode_std_timing(ctx, cnt - 16)?;
        } else if cnt == OFF_NUM_EXT {
            self.decode_descriptors(ctx, -72)?;
        } else if cnt == OFF_CHECKSUM {
            ctx.put_ann(
                ss,
                es,
                0,
                vec![format!("Extensions present: {}", cache_at(&self.cache, cnt - 1)?)],
            );
        } else if cnt == OFF_CHECKSUM + 1 {
            let mut checksum = 0;
            for i in 0..128 {
                checksum += cache_at(&self.cache, i)?;
            }
            let csstr = if checksum % 256 == 0 { "OK" } else { "WRONG!" };
            ctx.put_ann(
                ss,
                es,
                0,
                vec![format!("Checksum: {} ({csstr})", cache_at(&self.cache, cnt - 1)?)],
            );
            self.state = Some(State::Extensions);
        }
        Ok(())
    }

    fn decode_extension_byte(&mut self, ctx: &mut dyn Context, ss: u64, es: u64) -> Result<()> {
        let ext = usize::try_from(self.extension - 1)
            .map_err(|_| Error::device("edid: bad extension number"))?;
        let cache = self
            .ext_cache
            .get(ext)
            .ok_or_else(|| Error::device("edid: extension not seen"))?
            .clone();
        let v = cache_at(&cache, self.cnt - 1)?;
        if self.cnt == 1 {
            if v == 2 {
                ctx.put_ann(ss, es, 1, vec!["Extensions Tag".to_owned(), "Tag".to_owned()]);
            } else {
                ctx.put_ann(ss, es, 1, vec!["Bad Tag".to_owned()]);
            }
        } else if self.cnt == 2 {
            ctx.put_ann(ss, es, 1, vec!["Version".to_owned()]);
            ctx.put_ann(ss, es, 0, vec![v.to_string()]);
        } else if self.cnt == 3 {
            ctx.put_ann(ss, es, 1, vec!["DTD offset".to_owned()]);
            ctx.put_ann(ss, es, 0, vec![v.to_string()]);
        } else if self.cnt == 4 {
            ctx.put_ann(ss, es, 1, vec!["Format support | DTD count".to_owned()]);
            let support = format!(
                "Underscan: {}, {} Audio, YCbCr: {}",
                if v & 0x80 != 0 { "yes" } else { "no" },
                if v & 0x40 != 0 { "Basic" } else { "No" },
                ["None", "422", "444", "422+444"][((v & 0x30) >> 4) as usize]
            );
            ctx.put_ann(ss, es, 0, vec![format!("{support}, DTDs: {}", v & 0xf)]);
        } else {
            let dtd_offset = cache[2];
            if self.cnt <= dtd_offset {
                if self.cnt == dtd_offset {
                    let sn = self.ext_sn[ext].clone();
                    ctx.put_ann(
                        sample_at(&sn, 4)?.ss,
                        es,
                        1,
                        vec!["Data block collection".to_owned()],
                    );
                    let cache = self.ext_cache[ext].clone();
                    self.decode_data_block_collection(ctx, &cache[4..], &sn[4..])?;
                }
            } else if (self.cnt - dtd_offset) % 18 == 0 {
                let n = (self.cnt - dtd_offset) as f64 / 18.0;
                if n <= (cache[3] & 0xf) as f64 {
                    let sn = self.ext_sn[ext].clone();
                    ctx.put_ann(sample_at(&sn, self.cnt - 18)?.ss, es, 1, vec!["DTD".to_owned()]);
                    self.decode_descriptors(ctx, -18)?;
                }
            } else if self.cnt == 127 {
                let dtd_last = dtd_offset + (cache[3] & 0xf) * 18;
                let sn = self.ext_sn[ext].clone();
                ctx.put_ann(sample_at(&sn, dtd_last)?.ss, es, 1, vec!["Padding".to_owned()]);
            } else if self.cnt == 128 {
                let checksum = (self.ext_cache[ext].iter().sum::<i64>()) % 256;
                ctx.put_ann(
                    ss,
                    es,
                    0,
                    vec![format!(
                        "Checksum: {} ({})",
                        cache_at(&self.ext_cache[ext], self.cnt - 1)?,
                        if checksum != 0 { "Wrong" } else { "OK" }
                    )],
                );
            }
        }
        Ok(())
    }
}
