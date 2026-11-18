// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
//
// The order of the JTAG commands is openFPGALoader's (src/gowin.cpp and
// src/fsparser.cpp, Apache-2.0, by Gwenhael Goavec-Merou), which has it
// from Gowin's TN653 and UG290 and from what the parts turned out to need.

//! Loading a bitstream into a Gowin GW2A FPGA over JTAG: the `.fs` file,
//! and the commands that erase the configuration memory and write it.
//!
//! This is the load that is gone at power off. The board's flash, which
//! the FPGA configures itself from at power on, is not touched.
//!
//! A `.fs` file is text: a line of `0` and `1` per record, most
//! significant bit first, and comment lines that start with `//`. The
//! records up to the one whose key is `3b` are the header (the part's ID
//! code is in the one with key `06`); the rest is the configuration, a
//! line per row of the memory. All of it goes to the FPGA as it is, as
//! one long data register.

use crate::core::{Error, Result};

/// The JTAG instructions of the configuration logic, eight bits each.
mod ir {
    pub const NOOP: u8 = 0x02;
    pub const ERASE_SRAM: u8 = 0x05;
    pub const XFER_DONE: u8 = 0x09;
    pub const READ_IDCODE: u8 = 0x11;
    pub const INIT_ADDR: u8 = 0x12;
    pub const CONFIG_ENABLE: u8 = 0x15;
    pub const XFER_WRITE: u8 = 0x17;
    pub const CONFIG_DISABLE: u8 = 0x3a;
    pub const RELOAD: u8 = 0x3c;
    pub const STATUS: u8 = 0x41;
    /// Followed by the checksum as data, then [`CHECKSUM_END`].
    pub const CHECKSUM: u8 = 0x0a;
    pub const CHECKSUM_END: u8 = 0x08;
}

/// The status register.
mod status {
    pub const CRC_ERROR: u32 = 1 << 0;
    pub const MEMORY_ERASE: u32 = 1 << 5;
    pub const SYSTEM_EDIT_MODE: u32 = 1 << 7;
    pub const DONE_FINAL: u32 = 1 << 13;
}

/// The ID code of the GW2A-18 and GW2AR-18.
pub const IDCODE_GW2A_18: u32 = 0x0000_081b;
/// The version of the part is in the top four bits of an ID code.
const IDCODE_MASK: u32 = 0x0fff_ffff;
/// Rows of configuration the checksum covers on a GW2A-18.
const ROWS_GW2A_18: usize = 1342;
/// Looks at the status register a wait gets.
const POLLS: u32 = 2000;

/// The TAP of the FPGA, alone on its chain, at rest in Run-Test/Idle.
pub trait Tap {
    /// Shifts an eight bit instruction in.
    fn instruction(&mut self, ir: u8) -> Result<()>;
    /// Clocks in Run-Test/Idle.
    fn idle(&mut self, clocks: u32) -> Result<()>;
    /// Shifts 32 bits out of the data register.
    fn read32(&mut self) -> Result<u32>;
    /// Shifts `bits` bits into the data register, the first in bit 0 of
    /// the first byte.
    fn write(&mut self, data: &[u8], bits: usize) -> Result<()>;
}

impl Tap for crate::ftdi::mpsse::Jtag {
    fn instruction(&mut self, ir: u8) -> Result<()> {
        self.shift_ir(u32::from(ir), 8)
    }

    fn idle(&mut self, clocks: u32) -> Result<()> {
        crate::ftdi::mpsse::Jtag::idle(self, clocks)
    }

    fn read32(&mut self) -> Result<u32> {
        self.read_dr32()
    }

    fn write(&mut self, data: &[u8], bits: usize) -> Result<()> {
        self.write_dr(data, bits)
    }
}

/// A parsed `.fs` file.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Bitstream {
    /// The ID code of the part it was built for.
    pub idcode: u32,
    /// Every record, header and configuration, as the bytes the file has
    /// them in.
    pub data: Vec<u8>,
    /// The sum the FPGA is told, over the configuration rows.
    pub checksum: u32,
}

/// The value of a string of `0` and `1`, most significant first.
fn bits_value(line: &[u8]) -> u64 {
    line.iter().fold(0, |v, b| (v << 1) | u64::from(*b == b'1'))
}

impl Bitstream {
    /// Parses the text of a `.fs` file.
    ///
    /// # Errors
    ///
    /// Text that is no `.fs` file, or one without an ID code.
    pub fn parse(text: &[u8]) -> std::result::Result<Bitstream, String> {
        let mut lines: Vec<&[u8]> = Vec::new();
        for line in text.split(|b| *b == b'\n') {
            let line = line.strip_suffix(b"\r").unwrap_or(line);
            if line.is_empty() || line.starts_with(b"/") {
                continue;
            }
            if line.iter().any(|b| *b != b'0' && *b != b'1') {
                return Err("a line that is not zeros and ones".to_owned());
            }
            if !line.len().is_multiple_of(8) {
                return Err("a line that is not whole bytes".to_owned());
            }
            lines.push(line);
        }
        let mut idcode = None;
        let mut compressed = false;
        let mut crc = false;
        let mut rows = 0;
        let mut zeros = [0u8; 3];
        let mut header = None;
        for (i, line) in lines.iter().enumerate() {
            let key = (bits_value(&line[..8]) & 0x7f) as u8;
            let value = bits_value(&line[line.len().saturating_sub(64)..]);
            match (key, line.len()) {
                (0x06, 64) => idcode = Some(value as u32),
                (0x10, 64) => compressed = (value >> 13) & 1 != 0,
                // What stands for eight, four and two zero bytes in a
                // compressed row.
                (0x51, 64) => zeros = [(value >> 16) as u8, (value >> 8) as u8, value as u8],
                (0x3b, 32) => {
                    crc = (value >> 23) & 1 != 0;
                    rows = (value & 0xffff) as usize;
                    header = Some(i);
                    break;
                }
                (0x06 | 0x10 | 0x51 | 0x3b, _) => {
                    return Err(format!("a header record {key:02x} of the wrong length"));
                }
                _ => {}
            }
        }
        let header = header.ok_or("no end of the header")?;
        let idcode = idcode.ok_or("no ID code")?;
        let data: Vec<u8> =
            lines.iter().flat_map(|l| l.chunks(8)).map(|b| bits_value(b) as u8).collect();

        // The checksum: the configuration rows without the six bytes that
        // end each (eight with a CRC), compressed rows expanded, as 16 bit
        // words.
        let known = if idcode & IDCODE_MASK == IDCODE_GW2A_18 { ROWS_GW2A_18 } else { rows };
        let drop = if crc { 8 } else { 6 };
        let mut checksum = 0u32;
        let mut half: Option<u8> = None;
        for line in lines.iter().skip(header + 1).take(rows.min(known)) {
            let bytes = line.len() / 8;
            if bytes < drop {
                return Err("a configuration row that is too short".to_owned());
            }
            for chunk in line[..(bytes - drop) * 8].chunks(8) {
                let b = bits_value(chunk) as u8;
                let (b, repeat) = match zeros.iter().position(|z| compressed && *z == b) {
                    Some(i) => (0, 8 >> i),
                    None => (b, 1),
                };
                for _ in 0..repeat {
                    match half.take() {
                        Some(high) => {
                            checksum = checksum
                                .wrapping_add(u32::from(u16::from_be_bytes([high, b])))
                                & 0xffff;
                        }
                        None => half = Some(b),
                    }
                }
            }
        }
        Ok(Bitstream { idcode, data, checksum })
    }
}

fn command(tap: &mut dyn Tap, ir: u8) -> Result<()> {
    tap.instruction(ir)?;
    tap.idle(6)
}

fn register(tap: &mut dyn Tap, ir: u8) -> Result<u32> {
    command(tap, ir)?;
    tap.read32()
}

/// Reads the status until `mask` of it is `value`.
fn wait(tap: &mut dyn Tap, mask: u32, value: u32, what: &str) -> Result<()> {
    let mut last = 0;
    for _ in 0..POLLS {
        last = register(tap, ir::STATUS)?;
        if last & mask == value {
            return Ok(());
        }
    }
    Err(Error::device(format!("the FPGA did not get to {what}: its status is {last:#x}")))
}

/// The ID code of the FPGA on `tap`.
pub fn idcode(tap: &mut dyn Tap) -> Result<u32> {
    register(tap, ir::READ_IDCODE)
}

/// What a GW2A needs when its last configuration (from the flash, as a
/// rule) failed: undocumented, and without it the erase does not start.
fn clear_failure(tap: &mut dyn Tap) -> Result<()> {
    if register(tap, ir::STATUS)? & status::CRC_ERROR == 0 {
        return Ok(());
    }
    for _ in 0..2 {
        command(tap, ir::CONFIG_DISABLE)?;
        command(tap, 0)?;
        register(tap, ir::STATUS)?;
        idcode(tap)?;
    }
    command(tap, ir::CONFIG_ENABLE)?;
    command(tap, ir::RELOAD)?;
    command(tap, ir::NOOP)?;
    command(tap, ir::CONFIG_DISABLE)?;
    command(tap, ir::NOOP)?;
    idcode(tap)?;
    command(tap, ir::NOOP)?;
    idcode(tap)?;
    Ok(())
}

/// Loads `bitstream` into the configuration memory of the FPGA on `tap`,
/// which then runs it. What ran before is gone from the first command on.
///
/// # Errors
///
/// An FPGA that is not the part the bitstream is for, one that does not
/// follow the commands, or one that does not say it is done at the end.
pub fn load(tap: &mut dyn Tap, bitstream: &Bitstream) -> Result<()> {
    let part = idcode(tap)?;
    if part & IDCODE_MASK != IDCODE_GW2A_18 {
        return Err(Error::device(format!("the FPGA's ID code is {part:#010x}, not a GW2A-18's")));
    }
    if bitstream.idcode & IDCODE_MASK != part & IDCODE_MASK {
        return Err(Error::device(format!(
            "the bitstream is for ID code {:#010x}, the FPGA's is {part:#010x}",
            bitstream.idcode
        )));
    }
    clear_failure(tap)?;

    command(tap, ir::CONFIG_ENABLE)?;
    wait(tap, status::SYSTEM_EDIT_MODE, status::SYSTEM_EDIT_MODE, "its edit mode")?;
    command(tap, ir::ERASE_SRAM)?;
    command(tap, ir::NOOP)?;
    wait(tap, status::MEMORY_ERASE, status::MEMORY_ERASE, "the end of the erase")?;
    command(tap, ir::XFER_DONE)?;
    command(tap, ir::NOOP)?;
    command(tap, ir::CONFIG_DISABLE)?;
    command(tap, ir::NOOP)?;
    wait(tap, status::SYSTEM_EDIT_MODE, 0, "the end of its edit mode")?;

    command(tap, ir::CONFIG_ENABLE)?;
    command(tap, ir::INIT_ADDR)?;
    command(tap, ir::XFER_WRITE)?;
    // The file's first bit goes first, and JTAG sends bit 0 first.
    let data: Vec<u8> = bitstream.data.iter().map(|b| b.reverse_bits()).collect();
    tap.write(&data, data.len() * 8)?;
    command(tap, ir::CHECKSUM)?;
    tap.write(&bitstream.checksum.to_le_bytes(), 32)?;
    command(tap, ir::CHECKSUM_END)?;
    command(tap, ir::CONFIG_DISABLE)?;
    command(tap, ir::NOOP)?;

    let end = register(tap, ir::STATUS)?;
    if end & status::DONE_FINAL == 0 {
        return Err(Error::device(format!(
            "the FPGA did not take the bitstream: its status is {end:#x}"
        )));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn line(bytes: &[u8]) -> String {
        use std::fmt::Write as _;
        bytes.iter().fold(String::new(), |mut s, b| {
            write!(s, "{b:08b}").unwrap();
            s
        })
    }

    /// A file with `rows` rows of four bytes and the six that end a row.
    fn file(rows: &[[u8; 4]]) -> String {
        let mut text = String::from("//Tool: a test\r\n");
        text += &line(&[0xff; 4]);
        text += "\n";
        text += &line(&[0xa5, 0xc3]);
        text += "\n";
        text += &line(&[0x06, 0, 0, 0, 0x00, 0x00, 0x08, 0x1b]);
        text += "\n";
        text += &line(&[0x10, 0, 0, 0, 0, 0, 0, 0]);
        text += "\n";
        text += &line(&[0x3b, 0x00, 0x00, rows.len() as u8]);
        text += "\n";
        for row in rows {
            text += &line(row);
            text += &line(&[0xee; 6]);
            text += "\n";
        }
        text += &line(&[0xff; 2]);
        text += "\n";
        text
    }

    #[test]
    fn a_file_is_its_records_and_the_sum_of_its_rows() {
        let fs = Bitstream::parse(file(&[[0x12, 0x34, 0x00, 0x01], [0xff, 0xff, 0, 2]]).as_bytes())
            .unwrap();
        assert_eq!(fs.idcode, IDCODE_GW2A_18);
        // Comments are no part of it; every record is, the header too.
        assert_eq!(&fs.data[..6], &[0xff, 0xff, 0xff, 0xff, 0xa5, 0xc3]);
        assert_eq!(fs.data.len(), 4 + 2 + 8 + 8 + 4 + 2 * 10 + 2);
        // The rows without their last six bytes, as 16 bit words.
        assert_eq!(fs.checksum, (0x1234 + 0x0001 + 0xffff + 0x0002) & 0xffff);
    }

    #[test]
    fn what_is_no_bitstream_is_refused() {
        assert!(Bitstream::parse(b"hello\n").is_err());
        assert!(Bitstream::parse(b"0101\n").is_err());
        // No header end, and no ID code.
        assert!(Bitstream::parse(line(&[0x06, 0, 0, 0, 0, 0, 8, 0x1b]).as_bytes()).is_err());
        let no_id = file(&[]).replace(&line(&[0x06, 0, 0, 0, 0x00, 0x00, 0x08, 0x1b]), "");
        assert!(Bitstream::parse(no_id.as_bytes()).is_err());
    }

    /// An FPGA that follows the commands, and notes them.
    #[derive(Default)]
    struct Fpga {
        idcode: u32,
        ir: u8,
        status: u32,
        log: Vec<u8>,
        written: Vec<(u8, Vec<u8>, usize)>,
        /// Whether a write of configuration ends with the FPGA done.
        takes: bool,
    }

    impl Tap for Fpga {
        fn instruction(&mut self, ir: u8) -> Result<()> {
            self.ir = ir;
            if ir != super::ir::STATUS && ir != super::ir::READ_IDCODE {
                self.log.push(ir);
            }
            match ir {
                super::ir::CONFIG_ENABLE => self.status |= status::SYSTEM_EDIT_MODE,
                super::ir::CONFIG_DISABLE => self.status &= !status::SYSTEM_EDIT_MODE,
                super::ir::ERASE_SRAM => {
                    self.status = (self.status | status::MEMORY_ERASE) & !status::DONE_FINAL;
                }
                _ => {}
            }
            Ok(())
        }

        fn idle(&mut self, _clocks: u32) -> Result<()> {
            Ok(())
        }

        fn read32(&mut self) -> Result<u32> {
            Ok(if self.ir == super::ir::READ_IDCODE { self.idcode } else { self.status })
        }

        fn write(&mut self, data: &[u8], bits: usize) -> Result<()> {
            if self.ir == super::ir::XFER_WRITE && self.takes {
                self.status |= status::DONE_FINAL;
            }
            self.written.push((self.ir, data.to_vec(), bits));
            Ok(())
        }
    }

    fn bitstream() -> Bitstream {
        Bitstream { idcode: IDCODE_GW2A_18, data: vec![0x80, 0x01, 0xf0], checksum: 0xbeef }
    }

    #[test]
    fn a_load_erases_then_writes_the_file_first_bit_first() {
        let mut fpga = Fpga { idcode: IDCODE_GW2A_18, takes: true, ..Fpga::default() };
        load(&mut fpga, &bitstream()).unwrap();
        assert_eq!(
            fpga.log,
            [0x15, 0x05, 0x02, 0x09, 0x02, 0x3a, 0x02, 0x15, 0x12, 0x17, 0x0a, 0x08, 0x3a, 0x02]
        );
        assert_eq!(
            fpga.written,
            [(0x17, vec![0x01, 0x80, 0x0f], 24), (0x0a, vec![0xef, 0xbe, 0, 0], 32)]
        );
    }

    #[test]
    fn another_part_is_left_alone() {
        let mut fpga = Fpga { idcode: 0x0100_481b, takes: true, ..Fpga::default() };
        let e = load(&mut fpga, &bitstream()).unwrap_err().to_string();
        assert!(e.contains("0x0100481b"), "{e}");
        assert!(fpga.log.is_empty() && fpga.written.is_empty());

        let mut fpga = Fpga { idcode: IDCODE_GW2A_18, takes: true, ..Fpga::default() };
        let other = Bitstream { idcode: 0x0000_281b, ..bitstream() };
        assert!(load(&mut fpga, &other).is_err());
        assert!(fpga.log.is_empty() && fpga.written.is_empty());
    }

    #[test]
    fn an_fpga_that_is_not_done_is_a_failure() {
        let mut fpga = Fpga { idcode: IDCODE_GW2A_18, takes: false, ..Fpga::default() };
        let e = load(&mut fpga, &bitstream()).unwrap_err().to_string();
        assert!(e.contains("did not take"), "{e}");
    }
}
