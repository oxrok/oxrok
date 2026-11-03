// Counterpart in sigrok:
//   libsigrok/src/hardware/ftdi-la/api.c (the chips and the sequence),
//   libsigrok/src/hardware/ftdi-la/protocol.c (the sample rate and the
//   reads),
//   Copyright (C) 2015 Sergey Alirzaev <zl29ah@gmail.com>,
//   with commits by Uwe Hermann, Lars-Peter Clausen, Gerhard Sittig, Paul,
//   Sergey Rzhevsky, Tom Matthews
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it.

//! An FTDI chip used as a logic analyzer by sigrok's `ftdi-la` driver:
//! port A in asynchronous bit-bang mode with every pin an input, so that
//! each byte the chip sends the host is one sample of its eight data
//! pins, taken at a rate the baud rate divisor sets.
//!
//! The driver (through libftdi) opens the chip, resets it, sets 9600 Bd
//! (libftdi's open), purges both buffers, leaves and enters bit-bang
//! mode with mask 0, sets the baud rate to `samplerate / 20` on the H
//! chips or `/ 30` on the others (which libftdi multiplies by 4 in a
//! bit mode), enters bit-bang again when the acquisition starts and
//! reads 16 KiB at a time in bulk transfers of 4 KiB until it has its
//! samples.
//!
//! The pins carry a deterministic pattern: sample `n` is `n` modulo 256,
//! so `ADBUS0` toggles every sample and `ADBUS7` every 128. The count
//! starts over whenever the port enters a bit mode and when the host
//! purges the chip's buffer to it; a real chip keeps sampling into its
//! FIFO from the moment it enters bit-bang, but both tools then see the
//! same samples and the first one is the first after the driver's last
//! bit mode change. Every IN transfer is filled to the last packet,
//! as a chip whose FIFO is full answers; out of bit-bang mode the port
//! has nothing and answers the two status bytes alone.
//!
//! The checks on top of the shell's: the analyzer drives no pin (mask
//! 0), uses no bit mode but asynchronous bit-bang (or the reset back to
//! UART mode), writes nothing to the chip, and touches no port other
//! than A except through the kernel's `ftdi_sio`, which binds every
//! port at attach and reads its latency timer.
//!
//! Variants (`chip=`), the ones the driver's table names: `ft2232h`
//! (default, `0403:6010`), `ft4232h` (`0403:6011`), `ft232h`
//! (`0403:6014`), `ft232r` (`0403:6001`), `tumpa` (the TIAO USB
//! Multi-Protocol Adapter, an FT2232H at `0403:8a98`, which libftdi's
//! scan of the standard ids does not find, so the driver needs `conn=`
//! for it), and two the driver handles otherwise: `ft245bm` (a BM chip
//! at `0403:6001`, which the driver takes for an FT232R) and `ft230x`
//! (`0403:6015`, which libftdi lists and the driver rejects).
//! `manufacturer=`, `product=` and `serial=` replace the strings; an
//! empty value removes the string, as an unprogrammed EEPROM does.
//! The shell's faults ([`Faults`]) apply: `stall=`, `in-stall=`,
//! `idle=`.

use crate::model::{Descriptors, DeviceModel, Reply, Setup, Violations};
use crate::shells::ftdi::{
    self, Chip, Event, Faults, Fill, Ftdi, FtdiConfig, FtdiPorts, PortState, bitmode,
};
use crate::{Error, Result};

/// The model name.
pub const NAME: &str = "ftdi-la";
/// What `oxrok-emu list` says about it.
pub const ABOUT: &str = "FTDI chip in asynchronous bit-bang mode as sigrok's ftdi-la uses it; \
     chip=ft2232h|ft4232h|ft232h|ft232r|tumpa|ft245bm|ft230x, manufacturer=, product=, serial= \
     (empty: no string); the FTDI shell's faults: stall=REQUEST[/WVALUE], in-stall=N, idle=N";

/// The variants, by `chip=` name: chip, product id, manufacturer,
/// product, serial number.
const VARIANTS: &[(&str, Chip, u16, &str, &str, &str)] = &[
    ("ft2232h", Chip::Ft2232h, 0x6010, "FTDI", "Dual RS232-HS", ""),
    ("ft4232h", Chip::Ft4232h, 0x6011, "FTDI", "Quad RS232-HS", ""),
    ("ft232h", Chip::Ft232h, 0x6014, "FTDI", "Single RS232-HS", "FT6Q2Y3M"),
    ("ft232r", Chip::R, 0x6001, "FTDI", "FT232R USB UART", "A50285BI"),
    ("tumpa", Chip::Ft2232h, 0x8a98, "TIAO", "TIAO USB Multi-Protocol Adapter", "TIU1ZMV3"),
    ("ft245bm", Chip::Bm, 0x6001, "FTDI", "USB FIFO", "FTB4ZQ7K"),
    ("ft230x", Chip::Ft230x, 0x6015, "FTDI", "FT230X Basic UART", "DN01ABCD"),
];

/// A chip from the `-o` options.
///
/// # Errors
///
/// On an unknown option or chip.
pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>> {
    let mut config = config("ft2232h")?;
    let mut strings: Vec<(&str, &str)> = Vec::new();
    let mut faults = Faults::default();
    for (k, v) in options {
        match k.as_str() {
            "chip" => config = self::config(v)?,
            "manufacturer" | "product" | "serial" => strings.push((k, v)),
            other => {
                if !faults.option(other, v)? {
                    return Err(Error(format!("{NAME} has no option {other}")));
                }
            }
        }
    }
    for (k, v) in strings {
        let s = (!v.is_empty()).then(|| v.to_owned());
        match k {
            "manufacturer" => config.manufacturer = s,
            "product" => config.description = s,
            _ => config.serial = s,
        }
    }
    let mut model = FtdiLa::new(config);
    model.chip.set_faults(faults);
    Ok(Box::new(model))
}

/// The configuration of a `chip=` variant.
///
/// # Errors
///
/// When no variant has that name.
pub fn config(name: &str) -> Result<FtdiConfig> {
    let (_, chip, product, manufacturer, description, serial) =
        VARIANTS.iter().find(|v| v.0 == name).ok_or_else(|| {
            let names: Vec<&str> = VARIANTS.iter().map(|v| v.0).collect();
            Error(format!("chip={name}: one of {}", names.join(", ")))
        })?;
    let mut c = FtdiConfig::new(*chip);
    c.product = *product;
    c.manufacturer = Some((*manufacturer).to_owned());
    c.description = Some((*description).to_owned());
    c.serial = (!serial.is_empty()).then(|| (*serial).to_owned());
    Ok(c)
}

/// The pins behind the ports.
#[derive(Clone, Debug, Default)]
pub struct Pins {
    /// Samples handed out on each port since its last restart.
    pub sent: Vec<u64>,
}

impl Pins {
    /// Sample `n` of the pattern.
    pub fn sample(n: u64) -> u8 {
        n as u8
    }
}

impl FtdiPorts for Pins {
    fn changed(&mut self, port: usize, event: Event, state: &PortState, v: &mut Violations) {
        match event {
            Event::Bitmode => {
                if state.bitmode != bitmode::RESET {
                    self.sent[port] = 0;
                }
                if !matches!(state.bitmode, bitmode::RESET | bitmode::BITBANG) {
                    v.report(format!(
                        "bit mode {:#04x}: ftdi-la uses asynchronous bit-bang only",
                        state.bitmode
                    ));
                }
                if state.mask != 0 {
                    v.report(format!(
                        "bit mode output mask {:#04x}: a logic analyzer drives no pin",
                        state.mask
                    ));
                }
            }
            Event::FlushToHost => self.sent[port] = 0,
            _ => {}
        }
        if port != 0 && event != Event::Latency {
            v.report(format!(
                "{event:?} on port {}: ftdi-la samples port A only",
                ftdi::port_letter(port)
            ));
        }
    }

    fn write(&mut self, port: usize, data: &[u8], _: &PortState, v: &mut Violations) -> Reply {
        v.report(format!(
            "{} bytes written to port {}: ftdi-la writes nothing",
            data.len(),
            ftdi::port_letter(port)
        ));
        Reply::ack()
    }

    fn read(&mut self, port: usize, max: usize, state: &PortState, v: &mut Violations) -> Fill {
        if state.bitmode != bitmode::BITBANG {
            // A UART with nothing on its RX line.
            return Fill::Idle;
        }
        if port != 0 {
            v.report(format!(
                "read from port {}: ftdi-la samples port A only",
                ftdi::port_letter(port)
            ));
        }
        let start = self.sent[port];
        self.sent[port] += max as u64;
        Fill::Data((start..start + max as u64).map(Pins::sample).collect())
    }

    fn pins(&mut self, port: usize, _: &PortState, _: &mut Violations) -> u8 {
        Pins::sample(self.sent[port])
    }
}

/// The emulated chip.
pub struct FtdiLa {
    chip: Ftdi,
    pins: Pins,
}

impl FtdiLa {
    /// A chip of `config` at power on.
    pub fn new(config: FtdiConfig) -> FtdiLa {
        let ports = usize::from(config.chip.ports());
        FtdiLa { chip: Ftdi::new(config), pins: Pins { sent: vec![0; ports] } }
    }

    /// The shell, for the settings the host made.
    pub fn ftdi(&self) -> &Ftdi {
        &self.chip
    }
}

impl DeviceModel for FtdiLa {
    fn name(&self) -> &'static str {
        NAME
    }

    fn descriptors(&self) -> &Descriptors {
        self.chip.descriptors()
    }

    fn control(&mut self, setup: Setup, data: &[u8], v: &mut Violations) -> Reply {
        self.chip.control(&mut self.pins, setup, data, v)
    }

    fn out(&mut self, ep: u8, data: &[u8], v: &mut Violations) -> Reply {
        self.chip.out(&mut self.pins, ep, data, v)
    }

    fn in_transfer(&mut self, ep: u8, max: usize, v: &mut Violations) -> Reply {
        self.chip.in_transfer(&mut self.pins, ep, max, v)
    }

    fn reset(&mut self) {
        self.chip.reset();
        self.pins.sent.fill(0);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::shells::ftdi::libftdi_divisor;

    fn out(request: u8, value: u16, index: u16) -> Setup {
        Setup { request_type: 0x40, request, value, index, length: 0 }
    }

    /// What ftdi-la sends through libftdi: open, purge, bit-bang, the
    /// sample rate, bit-bang again, reads.
    #[test]
    fn samples_as_the_driver_reads_them() {
        let mut dev = FtdiLa::new(config("ft2232h").unwrap());
        let mut v = Violations::default();
        let (value, bits, _) = libftdi_divisor(Chip::Ft2232h, 9600);
        for s in [
            out(0, 0, 1),
            out(3, value, (bits << 8) | 1),
            out(0, 1, 1),
            out(0, 2, 1),
            out(0x0b, 0x0000, 1),
            out(0x0b, 0x0100, 1),
        ] {
            assert_eq!(dev.control(s, &[], &mut v), Reply::ack());
        }
        // 1 MHz: 50 kBd, times 4 in bit-bang mode.
        let (value, bits, _) = libftdi_divisor(Chip::Ft2232h, 200_000);
        assert_eq!(dev.control(out(3, value, (bits << 8) | 1), &[], &mut v), Reply::ack());
        assert_eq!(dev.ftdi().port(0).baud(), Some(200_000));
        // A read before the restart, then the restart.
        dev.in_transfer(1, 512, &mut v);
        assert_eq!(dev.control(out(0x0b, 0x0100, 1), &[], &mut v), Reply::ack());
        let Reply::Data(d) = dev.in_transfer(1, 4096, &mut v) else { panic!() };
        assert_eq!(d.len(), 4096);
        assert_eq!(&d[..4], &[0x01, 0x60, 0, 1]);
        assert_eq!(&d[512..515], &[0x01, 0x60, 254]);
        assert!(v.is_empty(), "{v:?}");
        dev.reset();
        assert_eq!(dev.in_transfer(1, 4096, &mut v), Reply::Data(vec![0x01, 0x60]));
    }

    #[test]
    fn flags_what_ftdi_la_never_does() {
        let mut dev = FtdiLa::new(config("ft2232h").unwrap());
        let mut v = Violations::default();
        dev.control(out(0x0b, 0x01ff, 1), &[], &mut v);
        assert_eq!(v.drain().len(), 1, "an output mask");
        dev.control(out(0x0b, 0x0200, 1), &[], &mut v);
        assert_eq!(v.drain().len(), 1, "MPSSE");
        dev.control(out(0x0b, 0x0100, 2), &[], &mut v);
        assert_eq!(v.drain().len(), 1, "port B");
        dev.in_transfer(3, 512, &mut v);
        assert_eq!(v.drain().len(), 1, "a read of port B");
        dev.out(2, &[0], &mut v);
        assert_eq!(v.drain().len(), 1, "a write");
        // The kernel reading port B's latency timer is fine.
        let s = Setup { request_type: 0xc0, request: 0x0a, value: 0, index: 2, length: 1 };
        assert_eq!(dev.control(s, &[], &mut v), Reply::Data(vec![16]));
        assert!(v.is_empty(), "{v:?}");
    }

    #[test]
    fn variants_and_strings() {
        for (name, chip, product, ..) in VARIANTS {
            let c = config(name).unwrap();
            assert_eq!((c.chip, c.product), (*chip, *product));
        }
        assert!(config("ft8u232am").is_err());
        let o = |k: &str, v: &str| (k.to_owned(), v.to_owned());
        let m = build(&[o("chip", "ft232r"), o("serial", ""), o("manufacturer", "ACME")]).unwrap();
        let d = m.descriptors();
        assert_eq!(d.strings, ["ACME", "FT232R USB UART"]);
        assert_eq!(&d.device[14..17], &[1, 2, 0]);
        assert!(build(&[o("speed", "1")]).is_err());
    }
}
