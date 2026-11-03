// Counterpart in sigrok:
//   libsigrok/src/hardware/asix-sigma/api.c (the ids, the scan by serial number),
//   libsigrok/src/hardware/asix-sigma/protocol.c (the bit-bang pins, the netlist scrambling, the FPGA configuration sequence),
//   libsigrok/src/hardware/asix-sigma/protocol.h,
//   Copyright (C) 2010-2012 Håvard Espeland <gus@ping.uio.no>,
//   Copyright (C) 2010 Martin Stensgård <mastensg@ping.uio.no>,
//   Copyright (C) 2010 Carl Henrik Lunde <chlunde@ping.uio.no>,
//   Copyright (C) 2020 Gerhard Sittig <gerhard.sittig@gmx.net>,
//   with commits by Uwe Hermann, Lars-Peter Clausen, Niklas Thorne, jry,
//   Daniel Trnka
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The ASIX SIGMA and SIGMA2 logic analyzers (and the OMEGA's scan) as
//! libsigrok's `asix-sigma` driver sees them: an FTDI chip in FIFO mode
//! in front of a Xilinx FPGA without configuration memory, which the
//! driver configures over the same pins in asynchronous bit-bang mode
//! every time it opens the device, then drives through a command
//! protocol ([`fpga`]).
//!
//! Facts and where they come from:
//!
//! - Ids `a600:a000` (SIGMA, SIGMA2) and `a600:a004` (OMEGA), and the
//!   serial number string that tells the models apart: hexadecimal,
//!   its upper 16 bits `a601` SIGMA, `a602` SIGMA2, `a603` OMEGA, which
//!   the driver skips ("not implemented") (`protocol.h`, `api.c` scan).
//!   The serial numbers themselves (`A6010A3C`, ...) are the model's.
//! - The chip: a single port FTDI chip used as an FT245 style FIFO and
//!   in asynchronous bit-bang mode, without MPSSE ("The FTDI cable is
//!   single channel", "the OMEGA device's cable contains a more capable
//!   FTDI chip and uses MPSSE mode", `protocol.c`). The model takes an
//!   FT245BM (`bcdDevice` 0x0400 with its serial number), as the brief
//!   for this model says; an FT245R would look the same to the driver,
//!   which uses nothing a BM lacks. The OMEGA's chip is taken as an
//!   FT232H, after the FT232H sections the driver's comment cites; the
//!   driver never opens one. Not verified against a device.
//! - Manufacturer and product strings (`ASIX`, `SIGMA`, `SIGMA2`,
//!   `OMEGA`) are the model's: the driver reads only the serial number
//!   (libftdi's open is given no description).
//! - The bit-bang pins (`protocol.c`, `BB_PIN_*`): D0 CCLK, D1 PROG, D2
//!   to D4 and D7 the "suicide sequence", D5 INIT (the only input, mask
//!   0xdf), D6 DIN. CCLK and PROG are inverted by the level shifters,
//!   so the FPGA samples DIN when the cable's CCLK falls and PROG set is
//!   `PROG_B` pulled low (Xilinx slave serial, UG332, as the driver's
//!   comments describe it). INIT set reads as `INIT_B` released.
//! - The netlists `asix-sigma-50.fw`, `-100.fw` and `-200.fw` are
//!   scrambled with the driver's xor sequence (`sigma_fw_2_bitbang`);
//!   the model unscrambles the stand-ins (`tools/emu/asix-sigma.firmware`,
//!   sizes the model's own) and holds the bits clocked in against them,
//!   so an upload is checked bit for bit. `-50sync.fw` and
//!   `-phasor.fw` are never chosen by the driver.
//!
//! The FPGA's configuration, as the model plays it: `PROG_B` pulled low
//! clears it (`INIT_B` low); once `PROG_B` is released `INIT_B` stays low for
//! `init-busy=N` bit-bang reads (0 by default: clearing is over by the
//! time the driver reads, after its 10 ms sleep), then rises, and the
//! bits that follow are the bitstream. When the host leaves bit-bang
//! mode the bits are compared with the netlists; one that matches is
//! running from then on. A configured FPGA also drops its netlist on
//! the suicide sequence (D7 held, D2 and D3 alternating). In bit-bang
//! mode the chip samples the pins at the baud rate, so every read is a
//! full transfer of samples, as in the `ftdi-la` model: the outputs as
//! last written, and INIT. Out of bit-bang mode a configured FPGA
//! answers its commands through the FIFO; an unconfigured one takes
//! nothing and answers nothing.
//!
//! The checks on top of the shell's and the FPGA's: bit-bang only, with
//! mask 0xdf (D5 is the FPGA's open drain `INIT_B`), pin states the
//! driver writes and no others, DIN steady across each CCLK edge, a
//! bitstream that matches a netlist.
//!
//! Options: `variant=sigma|sigma2|omega`, `serial=` (empty: no serial
//! number string), `product=N` (another product id of ASIX's, which
//! the driver's scan skips), `step=N` (samples per count of the pattern, see
//! [`fpga`]); faults, all off by default: `init-busy=N` (above),
//! `config=fail` (the FPGA rejects the bitstream, as on a CRC error,
//! and stays unconfigured), `id=N` (what `READ_ID` answers, 0xa6),
//! `scratch=stuck` (the scratch register reads 0xff, as libsigrok saw
//! unused registers read) or `scratch=bit0` (its bit 0 stuck at 1), `stop-polls=N` (mode reads before a forced
//! stop is over), and the FTDI shell's `stall=`, `in-stall=`, `idle=`.
//!
//! What the cases (`tools/emu/asix-sigma.sh`) leave unreached in the C
//! driver (`tests/emu/asix-sigma/gcov.txt`), and why:
//!
//! - Scan: `libusb_open` or the serial string read failing (`api.c`
//!   182-194); the device opens and answers its descriptors.
//! - The OMEGA's open (`dev_open` 277-278): the scan skips it.
//! - Guards against NULL arguments and the driver's own buffer size
//!   checks (`config_get` 302, `config_list` 409/417, 73/76/82,
//!   159/162, 268-269, 320-321, 393-394, 1046, 1312, 1331, 1352, 1376,
//!   1697, 1760, 1228/1232).
//! - Allocation failures (86-88, 798-800, 1177, 1456, 2033) and
//!   failures of the session or of the software limits (476, 617, 623,
//!   948, 963, 988, 1204, 1215, 1254, 1286, 1728/1731, 1750, 2037,
//!   2040, 2077, 2090, 2096); `sigma_build_basic_trigger` and the
//!   samplerate normalisation never fail where checked (`api.c` 515,
//!   1098).
//! - Defaults for keys sigrok-cli never passes (`api.c` 329-330,
//!   390-391, 428-429).
//! - `sigma_check_open`/`_close`: called only from the dead code after
//!   `sigma_fetch_hw_config`'s `return SR_ERR_NA`.
//! - A missing firmware file (769, 872-873, the host's files, not the
//!   device's), a bad firmware index (835, 838), "not uploading again"
//!   (842-843: a second acquisition in one session, which sigrok-cli
//!   never makes).
//! - A short bulk write (214): libusb does not return one here.
//! - A failed write of the netlist's first commands (724-725): a
//!   control request precedes it, which an unplug fails first.
//! - A rewind across row 0 (1345): a wrapped capture with its trigger
//!   in the first events of row 0.
//! - A trigger on a disabled channel (1629): sigrok-cli refuses it.
//! - A stop asked for during a capture (`api.c` 644, 2182, 2192, 2198):
//!   sigrok-cli has no SIGINT handler.
//! - The trigger functions the driver never builds: a second edge or a
//!   function other than OR (457, 475-481, 2259-2325, 2371-2387); two
//!   edges are refused before.

mod fpga;

use crate::model::{Descriptors, DeviceModel, Reply, Setup, Violations};
use crate::shells::ezusb;
use crate::shells::ftdi::{
    Chip, Countdown, Event, Faults, Fill, Ftdi, FtdiConfig, FtdiPorts, PortState, bitmode,
};
use crate::txlog::HASH_ABOVE;
use crate::{Error, Result};

pub use fpga::{Fpga, FpgaFaults, Netlist};

/// The model name.
pub const NAME: &str = "asix-sigma";
/// What `oxrok-emu list` says about it.
pub const ABOUT: &str = "ASIX SIGMA/SIGMA2 (FT245BM and an FPGA configured over bit-bang); \
     variant=sigma|sigma2|omega, serial= (empty: none), product=N, step=N; faults: \
     init-busy=N, config=fail, id=N, scratch=stuck|bit0, stop-polls=N, the FTDI shell's stall=, in-stall=, idle=";

/// ASIX's vendor id.
pub const VENDOR: u16 = 0xa600;

/// The variants: name, product id, chip, product string, serial number.
const VARIANTS: &[(&str, u16, Chip, &str, &str)] = &[
    ("sigma", 0xa000, Chip::Bm, "SIGMA", "A6010A3C"),
    ("sigma2", 0xa000, Chip::Bm, "SIGMA2", "A6020B71"),
    ("omega", 0xa004, Chip::Ft232h, "OMEGA", "A6030D52"),
];

/// The netlists the driver uploads: file, stand-in size, netlist.
pub const NETLISTS: &[(&str, usize, Netlist)] = &[
    ("asix-sigma-50.fw", 2000, Netlist::Mhz50),
    ("asix-sigma-100.fw", 2100, Netlist::Mhz100),
    ("asix-sigma-200.fw", 2200, Netlist::Mhz200),
];

/// Bit-bang pins (`BB_PIN_*`).
mod pin {
    pub const CCLK: u8 = 1 << 0;
    pub const PROG: u8 = 1 << 1;
    pub const D2: u8 = 1 << 2;
    pub const D3: u8 = 1 << 3;
    pub const INIT: u8 = 1 << 5;
    pub const DIN: u8 = 1 << 6;
    pub const D7: u8 = 1 << 7;
    /// Every pin but INIT is an output.
    pub const MASK: u8 = 0xdf;
}

/// The pin states the driver writes: the suicide sequence, the PROG
/// pulse, and the bitstream's CCLK and DIN.
const PIN_STATES: [u8; 7] = [0x84, 0x88, 0x01, 0x03, 0x41, 0x40, 0x00];

/// A stand-in netlist as the FPGA takes it: the file unscrambled
/// (`sigma_fw_2_bitbang`'s xor sequence).
pub fn netlist_bits(name: &str, size: usize) -> Vec<u8> {
    let mut data = ezusb::standin(name, size);
    let mut imm: u32 = 0x3f6d_f2ab;
    for b in &mut data {
        imm = (imm.wrapping_add(0x0a85_3753) % 177).wrapping_add(imm.wrapping_mul(0x0803_4052));
        *b ^= imm as u8;
    }
    data
}

/// A device from the `-o` options.
///
/// # Errors
///
/// On an unknown option or a bad value.
pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>> {
    let mut variant = VARIANTS[0];
    let mut product = None;
    let mut serial: Option<Option<String>> = None;
    let mut step = 2;
    let mut faults = Faults::default();
    let mut board = BoardFaults::default();
    let number = |k: &str, v: &str| -> Result<u64> {
        let n = match v.strip_prefix("0x") {
            Some(h) => u64::from_str_radix(h, 16),
            None => v.parse(),
        };
        n.map_err(|_| Error(format!("{k}={v}: not a number")))
    };
    for (k, v) in options {
        match k.as_str() {
            "variant" => {
                variant = *VARIANTS.iter().find(|x| x.0 == v).ok_or_else(|| {
                    Error(format!("variant={v}: sigma, sigma2 or omega"))
                })?;
            }
            "product" => {
                product = Some(
                    u16::try_from(number(k, v)?)
                        .map_err(|_| Error(format!("product={v}: 16 bits")))?,
                );
            }
            "serial" => serial = Some((!v.is_empty()).then(|| v.clone())),
            "step" => {
                step = number(k, v)?;
                if step == 0 {
                    return Err(Error("step=0: at least 1".into()));
                }
            }
            "init-busy" => board.init_busy = number(k, v)? as u32,
            "stop-polls" => board.fpga.stop_polls = number(k, v)? as u32,
            "id" => {
                board.fpga.id = u8::try_from(number(k, v)?)
                    .map_err(|_| Error(format!("id={v}: one byte")))?;
            }
            "scratch" if v == "stuck" => board.fpga.scratch_stuck = 0xff,
            "scratch" if v == "bit0" => board.fpga.scratch_stuck = 0x01,
            "config" if v == "fail" => board.config_fails = true,
            "scratch" | "config" => {
                return Err(Error(format!("{k}={v}: scratch=stuck|bit0 or config=fail")));
            }
            other => {
                if !faults.option(other, v)? {
                    return Err(Error(format!("{NAME} has no option {other}")));
                }
            }
        }
    }
    let (_, variant_product, chip, description, default_serial) = variant;
    let product = product.unwrap_or(variant_product);
    let mut config = FtdiConfig::new(chip);
    config.vendor = VENDOR;
    config.product = product;
    config.manufacturer = Some("ASIX".to_owned());
    config.description = Some(description.to_owned());
    config.serial = serial.unwrap_or_else(|| Some(default_serial.to_owned()));
    let mut model = Sigma::new(config, board, step);
    model.chip.set_faults(faults);
    Ok(Box::new(model))
}

/// The board's own faults.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct BoardFaults {
    /// Bit-bang reads with `INIT_B` low after `PROG_B` is released.
    pub init_busy: u32,
    /// The FPGA rejects every bitstream.
    pub config_fails: bool,
    /// The FPGA's.
    pub fpga: FpgaFaults,
}

/// Where the FPGA's configuration stands.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Init {
    /// `INIT_B` high: taking bits on CCLK.
    Ready,
    /// `INIT_B` low: `PROG_B` held or the memory being cleared.
    Clearing,
}

/// The pins behind the FTDI chip: the FPGA and its configuration.
#[derive(Clone, Debug)]
pub struct Board {
    faults: BoardFaults,
    step: u64,
    /// The pins as last written in bit-bang mode.
    pins: u8,
    init: Init,
    clearing: Countdown,
    /// The bits clocked in since the last PROG pulse, MSB first.
    bits: Vec<u8>,
    nbits: usize,
    /// D2/D3 alternations with D7 held, for the suicide sequence.
    suicide: u32,
    fpga: Option<Fpga>,
}

impl Board {
    fn new(faults: BoardFaults, step: u64) -> Board {
        Board {
            faults,
            step,
            pins: 0,
            // At power on the FPGA clears itself and waits for a
            // configuration.
            init: Init::Ready,
            clearing: Countdown::new(faults.init_busy),
            bits: Vec::new(),
            nbits: 0,
            suicide: 0,
            fpga: None,
        }
    }

    /// The FPGA, once configured.
    pub fn fpga(&self) -> Option<&Fpga> {
        self.fpga.as_ref()
    }

    fn pin_state(&mut self, b: u8, v: &mut Violations) {
        if !PIN_STATES.contains(&b) {
            v.report(format!(
                "bit-bang pin state {b:#04x}: the driver writes {PIN_STATES:02x?} (suicide, PROG \
                 pulse, CCLK and DIN)"
            ));
        }
        let prev = self.pins;
        self.pins = b;
        // The suicide sequence: D7 held, D2 and D3 taking turns.
        let turn = |x: u8| x & pin::D7 != 0 && (x & (pin::D2 | pin::D3)).is_power_of_two();
        if turn(b) && turn(prev) && (b ^ prev) & (pin::D2 | pin::D3) == pin::D2 | pin::D3 {
            self.suicide += 1;
            if self.suicide >= 4 {
                self.fpga = None;
            }
        } else if !turn(b) {
            self.suicide = 0;
        }
        let (prog, was_prog) = (b & pin::PROG != 0, prev & pin::PROG != 0);
        if prog && !was_prog {
            // `PROG_B` low: the configuration is gone.
            self.fpga = None;
            self.init = Init::Clearing;
            self.bits.clear();
            self.nbits = 0;
        }
        if !prog && was_prog {
            if self.faults.init_busy == 0 {
                self.init = Init::Ready;
            } else {
                self.clearing.arm();
            }
        }
        // The cable's CCLK falling is the FPGA's rising edge.
        if prev & pin::CCLK != 0 && b & pin::CCLK == 0 && !prog {
            // While `INIT_B` is low the FPGA takes no bit (the driver's
            // retry clocks one from its PROG pulse into the suicide).
            if self.init == Init::Clearing {
                return;
            }
            if (prev ^ b) & pin::DIN != 0 {
                v.report("DIN changes at the CCLK edge that samples it");
            }
            if self.nbits.is_multiple_of(8) {
                self.bits.push(0);
            }
            if b & pin::DIN != 0 {
                *self.bits.last_mut().expect("pushed") |= 0x80 >> (self.nbits % 8);
            }
            self.nbits += 1;
        }
    }

    /// The host left bit-bang mode: what was clocked in is a netlist,
    /// or nothing.
    fn configure(&mut self, v: &mut Violations) {
        if self.nbits == 0 {
            return;
        }
        let found = NETLISTS.iter().find(|&&(name, size, _)| {
            self.nbits == size * 8 && self.bits == netlist_bits(name, size)
        });
        match found {
            Some(&(_, _, netlist)) => {
                if !self.faults.config_fails {
                    self.fpga = Some(Fpga::new(netlist, self.faults.fpga, self.step));
                }
            }
            None => v.report(format!(
                "a bitstream of {} bits that is none of the netlists {:?}",
                self.nbits,
                NETLISTS.iter().map(|n| n.0).collect::<Vec<_>>()
            )),
        }
        self.bits.clear();
        self.nbits = 0;
    }

    /// The pins as the chip samples them.
    fn sample(&mut self) -> u8 {
        if self.init == Init::Clearing && self.pins & pin::PROG == 0 && !self.clearing.busy() {
            self.init = Init::Ready;
        }
        let init = if self.init == Init::Ready && self.pins & pin::PROG == 0 { pin::INIT } else { 0 };
        (self.pins & pin::MASK) | init
    }
}

impl FtdiPorts for Board {
    fn changed(&mut self, _: usize, event: Event, state: &PortState, v: &mut Violations) {
        match event {
            Event::Bitmode => match state.bitmode {
                bitmode::RESET => self.configure(v),
                bitmode::BITBANG => {
                    if state.mask != pin::MASK {
                        v.report(format!(
                            "bit-bang output mask {:#04x}: 0xdf, D5 is the FPGA's INIT_B",
                            state.mask
                        ));
                    }
                }
                other => v.report(format!(
                    "bit mode {other:#04x}: the driver uses asynchronous bit-bang only"
                )),
            },
            Event::FlushToHost | Event::Reset => {
                if let Some(f) = self.fpga.as_mut() {
                    f.purge();
                }
            }
            _ => {}
        }
    }

    fn write(&mut self, _: usize, data: &[u8], state: &PortState, v: &mut Violations) -> Reply {
        if state.bitmode == bitmode::BITBANG {
            for &b in data {
                self.pin_state(b, v);
            }
        } else if let Some(f) = self.fpga.as_mut() {
            f.commands(data, v);
        }
        // An unconfigured FPGA takes nothing from the FIFO.
        Reply::ack()
    }

    fn read(&mut self, _: usize, max: usize, state: &PortState, _: &mut Violations) -> Fill {
        if state.bitmode == bitmode::BITBANG {
            return Fill::Data(vec![self.sample(); max]);
        }
        match self.fpga.as_mut() {
            Some(f) if f.has_output() => Fill::Data(f.take(max)),
            _ => Fill::Idle,
        }
    }

    fn pins(&mut self, _: usize, _: &PortState, _: &mut Violations) -> u8 {
        self.sample()
    }
}

/// The emulated device.
pub struct Sigma {
    chip: Ftdi,
    board: Board,
    initial: Board,
}

impl Sigma {
    /// A device of `config` at power on.
    pub fn new(config: FtdiConfig, faults: BoardFaults, step: u64) -> Sigma {
        let board = Board::new(faults, step);
        Sigma { chip: Ftdi::new(config), initial: board.clone(), board }
    }

    /// The pins and the FPGA.
    pub fn board(&self) -> &Board {
        &self.board
    }
}

impl DeviceModel for Sigma {
    fn name(&self) -> &'static str {
        NAME
    }

    fn descriptors(&self) -> &Descriptors {
        self.chip.descriptors()
    }

    fn control(&mut self, setup: Setup, data: &[u8], v: &mut Violations) -> Reply {
        self.chip.control(&mut self.board, setup, data, v)
    }

    fn out(&mut self, ep: u8, data: &[u8], v: &mut Violations) -> Reply {
        self.chip.out(&mut self.board, ep, data, v)
    }

    fn in_transfer(&mut self, ep: u8, max: usize, v: &mut Violations) -> Reply {
        self.chip.in_transfer(&mut self.board, ep, max, v)
    }

    fn reset(&mut self) {
        self.chip.reset();
        self.board = self.initial.clone();
    }

    fn log_digest_above(&self) -> Option<usize> {
        Some(HASH_ABOVE)
    }

    fn take_event_point(&mut self) -> bool {
        self.board.fpga.as_mut().is_some_and(Fpga::take_event_point)
    }
}

#[cfg(test)]
mod tests;
