//! oxrok: a sigrok compatible stack in Rust.
//!
//! One crate, organised as the layers a device goes through:
//!
//! - [`core`]: keys, values, packets, the device and driver contracts,
//!   the session and the trigger model;
//! - transports: [`usb`], [`serial`], [`hid`], [`scpi`], [`ftdi`] and
//!   [`ezusb`], each with record and replay for tests;
//! - [`drivers`]: one module per device family;
//! - [`dmm`]: the multimeter packet parsers;
//! - [`formats`]: input and output modules byte compatible with
//!   sigrok-cli;
//! - [`decode`] and [`decoders`]: the protocol decoder runtime and the
//!   native decoders; [`pydecode`] hosts libsigrokdecode's Python
//!   decoders when built with the `rustpython` or `python` feature.
//!
//! # Features
//!
//! - `cli` (default): the `oxrok` command line tool.
//! - `nusb` (default) and `libusb`: the USB backends; with both,
//!   `OXROK_USB_BACKEND` picks one at run time.
//! - `rustpython`: [`pydecode`], which runs libsigrokdecode's Python
//!   decoders unchanged, on a RustPython interpreter built into the
//!   binary. Needs Rust 1.96.
//! - `python`: the same on the system's CPython, through pyo3. With
//!   both, CPython is used unless `OXROK_PYTHON=rustpython`.
//!
//! # Installing
//!
//! ```text
//! cargo install oxrok --version 0.1.0-alpha.1 --locked
//! cargo install oxrok --version 0.1.0-alpha.1 --locked --features rustpython
//! cargo install oxrok --version 0.1.0-alpha.1 --locked --features python
//! ```
//!
//! The release archives on GitHub hold a `rustpython` build with the
//! libsigrokdecode 0.5.3 decoders next to it, and need nothing else.
//!
//! The `rustpython` feature relies on a pin in `Cargo.lock`, so install
//! with `--locked`. As a dependency, pin it in your own lockfile:
//! `cargo update -p malachite-bigint@0.14.0 --precise 0.12.0`.
//!
//! The `python` feature needs Python 3.9 or newer with its shared
//! library at build and at run time (Homebrew or python.org Pythons,
//! `python3-dev` on Debian and Ubuntu, `python3-devel` on Fedora), and
//! the decoders of a libsigrokdecode installation (`brew install
//! libsigrokdecode`, the distribution package, or a checkout). The build
//! takes the interpreter from `PYO3_PYTHON`, the active virtualenv, or
//! `python` and `python3` on `PATH`. With either feature, decoders are
//! found in `OXROK_DECODERS_DIR`, `SIGROKDECODE_DIR`, `decoders` next to
//! the executable, `~/.local/share/libsigrokdecode/decoders` and the
//! usual system directories.
//!
//! As a library dependency:
//!
//! ```toml
//! [dependencies]
//! oxrok = { version = "0.1.0-alpha.1", default-features = false, features = ["nusb"] }
//! ```
//!
//! Every module forbids unsafe code except `pydecode`, which crosses
//! into Python.

pub mod core;
pub mod decode;
pub mod decoders;
pub mod dmm;
pub mod ezusb;
pub mod formats;
pub mod ftdi;
pub mod hid;
pub mod modbus;
#[cfg(any(feature = "python", feature = "rustpython"))]
pub mod pydecode;
pub mod scpi;
pub mod serial;
pub mod usb;

/// The device drivers, one module per family.
pub mod drivers {
    pub mod agilent_dmm;
    pub mod asix_sigma;
    pub mod atten_pps3xxx;
    pub mod chronovu_la;
    pub mod dcttech_usbrelay;
    pub mod demo;
    pub mod dreamsourcelab_dslogic;
    pub mod fluke_dmm;
    pub mod ftdi_la;
    pub mod fx2lafw;
    pub mod greatfet;
    pub mod gwinstek_gpd;
    pub mod gwinstek_psp;
    pub mod hantek_4032l;
    pub mod hantek_6xxx;
    pub mod hantek_dso;
    pub mod icstation_usbrelay;
    pub mod ikalogic_scanalogic2;
    pub mod ikalogic_scanaplus;
    pub mod itech_it8500;
    pub mod juntek_jds6600;
    pub mod kecheng_kc_330b;
    pub mod kingst_la2016;
    pub mod korad_kaxxxxp;
    pub mod lascar_el_usb;
    pub mod lecroy_logicstudio;
    pub mod logicport;
    pub mod manson_hcs_3xxx;
    pub mod maynuo_m97;
    pub mod microchip_pickit2;
    pub mod motech_lps_30x;
    pub mod ols;
    pub mod pipistrello_ols;
    pub mod raspberrypi_pico;
    pub mod rdtech_dps;
    pub mod rdtech_tc;
    pub mod rdtech_um;
    pub mod rigol_ds;
    pub mod rohde_schwarz_sme_0x;
    pub mod saleae_logic16;
    pub mod saleae_logic_pro;
    pub mod serial_dmm;
    pub mod sysclk_lwla;
    pub mod sysclk_sla5032;
    pub mod testo;
    pub mod uni_t_dmm;
    pub mod zeroplus_logic_cube;
    pub mod zketech_ebd_usb;
}

/// Registers every driver.
pub fn register_all(registry: &mut core::Registry) {
    registry.register(Box::new(drivers::demo::DemoDriver));
    registry.register(Box::new(drivers::logicport::LogicPortDriver));
    drivers::serial_dmm::register_all(registry);
    registry.register(Box::new(drivers::juntek_jds6600::JdsDriver));
    registry.register(Box::new(drivers::korad_kaxxxxp::KoradDriver));
    registry.register(Box::new(drivers::fluke_dmm::FlukeDriver));
    registry.register(Box::new(drivers::agilent_dmm::AgilentDmmDriver));
    registry.register(Box::new(drivers::atten_pps3xxx::AttenPpsDriver));
    registry.register(Box::new(drivers::itech_it8500::ItechIt8500Driver));
    registry.register(Box::new(drivers::ols::OlsDriver));
    registry.register(Box::new(drivers::rdtech_um::RdtechUmDriver));
    registry.register(Box::new(drivers::manson_hcs_3xxx::MansonDriver));
    registry.register(Box::new(drivers::gwinstek_gpd::GpdDriver));
    registry.register(Box::new(drivers::motech_lps_30x::MotechLps301Driver));
    registry.register(Box::new(drivers::gwinstek_psp::PspDriver));
    registry.register(Box::new(drivers::rdtech_dps::RdtechDpsDriver));
    registry.register(Box::new(drivers::rdtech_dps::RdtechRdDriver));
    registry.register(Box::new(drivers::maynuo_m97::MaynuoM97Driver));
    registry.register(Box::new(drivers::rigol_ds::RigolDsDriver));
    registry.register(Box::new(drivers::rohde_schwarz_sme_0x::SmeDriver));
    registry.register(Box::new(drivers::raspberrypi_pico::RaspberryPiPicoDriver));
    registry.register(Box::new(drivers::rdtech_tc::RdtechTcDriver));
    registry.register(Box::new(drivers::dcttech_usbrelay::UsbRelayDriver));
    registry.register(Box::new(drivers::microchip_pickit2::Pickit2Driver));
    registry.register(Box::new(drivers::fx2lafw::Fx2lafwDriver));
    registry.register(Box::new(drivers::saleae_logic16::Logic16Driver));
    registry.register(Box::new(drivers::saleae_logic_pro::LogicProDriver));
    registry.register(Box::new(drivers::ftdi_la::FtdiLaDriver));
    registry.register(Box::new(drivers::testo::TestoDriver));
    registry.register(Box::new(drivers::icstation_usbrelay::IcstationUsbrelayDriver));
    registry.register(Box::new(drivers::ikalogic_scanalogic2::Scanalogic2Driver));
    registry.register(Box::new(drivers::ikalogic_scanaplus::ScanaPlusDriver));
    registry.register(Box::new(drivers::asix_sigma::AsixSigmaDriver));
    registry.register(Box::new(drivers::chronovu_la::ChronovuLaDriver));
    registry.register(Box::new(drivers::kecheng_kc_330b::Kc330bDriver));
    registry.register(Box::new(drivers::kingst_la2016::La2016Driver));
    registry.register(Box::new(drivers::pipistrello_ols::PipistrelloOlsDriver));
    registry.register(Box::new(drivers::dreamsourcelab_dslogic::DslogicDriver));
    registry.register(Box::new(drivers::hantek_dso::HantekDsoDriver));
    registry.register(Box::new(drivers::hantek_6xxx::Hantek6xxxDriver));
    registry.register(Box::new(drivers::greatfet::GreatfetDriver));
    registry.register(Box::new(drivers::sysclk_lwla::LwlaDriver));
    registry.register(Box::new(drivers::lecroy_logicstudio::LogicStudioDriver));
    registry.register(Box::new(drivers::hantek_4032l::Hantek4032lDriver));
    registry.register(Box::new(drivers::zeroplus_logic_cube::ZeroplusDriver));
    registry.register(Box::new(drivers::zketech_ebd_usb::ZketechEbdUsbDriver));
    registry.register(Box::new(drivers::lascar_el_usb::LascarDriver));
    registry.register(Box::new(drivers::sysclk_sla5032::Sla5032Driver));
}
