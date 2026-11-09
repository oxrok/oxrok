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
//! cargo install oxrok --version 0.2.0 --locked
//! cargo install oxrok --version 0.2.0 --locked --features rustpython
//! cargo install oxrok --version 0.2.0 --locked --features python
//! ```
//!
//! The archives of a GitHub release, where it has them, hold a `rustpython`
//! build with the libsigrokdecode 0.5.3 decoders next to it, and need nothing
//! else.
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
//! oxrok = { version = "0.2.0", default-features = false, features = ["nusb"] }
//! ```
//!
//! Every module forbids unsafe code except `pydecode`, which crosses
//! into Python; `gpib`, whose ioctls on `/dev/gpibN` are unsafe calls; and
//! `spawn`, which marks inherited descriptors close-on-exec by their numbers
//! before a driver starts a program.

pub mod core;
pub mod decode;
pub mod decoders;
pub mod dmm;
pub mod ezusb;
pub mod formats;
pub mod ftdi;
pub mod gpib;
pub mod hid;
pub mod lcr;
pub mod modbus;
#[cfg(any(feature = "python", feature = "rustpython"))]
pub mod pydecode;
pub mod scpi;
pub mod serial;
pub mod spawn;
pub mod usb;

/// The device drivers, one module per family.
pub mod drivers {
    pub mod agilent_dmm;
    pub mod appa_55ii;
    pub mod arachnid_labs_re_load_pro;
    pub mod asix_omega_rtm_cli;
    pub mod asix_sigma;
    pub mod atorch;
    pub mod atten_pps3xxx;
    #[cfg(target_os = "linux")]
    pub mod baylibre_acme;
    pub mod beaglelogic;
    pub mod bkprecision_1856d;
    pub mod cem_dt_885x;
    pub mod center_3xx;
    pub mod chronovu_la;
    pub mod colead_slm;
    pub mod conrad_digi_35_cpu;
    pub mod dcttech_usbrelay;
    pub mod demo;
    pub mod devantech_eth008;
    pub mod dreamsourcelab_dslogic;
    pub mod fluke_45;
    pub mod fluke_dmm;
    pub mod ftdi_la;
    pub mod fx2lafw;
    pub mod gmc_mh_1x_2x;
    pub mod greatfet;
    pub mod gwinstek_gds_800;
    pub mod gwinstek_gpd;
    pub mod gwinstek_psp;
    pub mod hameg_hmo;
    pub mod hantek_4032l;
    pub mod hantek_6xxx;
    pub mod hantek_dso;
    pub mod hp_3457a;
    pub mod hp_3478a;
    pub mod hp_59306a;
    pub mod icstation_usbrelay;
    pub mod ikalogic_scanalogic2;
    pub mod ikalogic_scanaplus;
    pub mod ipdbg_la;
    pub mod itech_it8500;
    pub mod juntek_jds6600;
    pub mod kecheng_kc_330b;
    pub mod kern_scale;
    pub mod kingst_la2016;
    pub mod korad_kaxxxxp;
    pub mod lascar_el_usb;
    pub mod lecroy_logicstudio;
    pub mod lecroy_xstream;
    pub mod link_mso19;
    pub mod logicport;
    pub mod manson_hcs_3xxx;
    pub mod mastech_ms6514;
    pub mod maynuo_m97;
    pub mod mic_985xx;
    pub mod microchip_pickit2;
    pub mod motech_lps_30x;
    pub mod norma_dmm;
    pub mod ols;
    pub mod pce_322a;
    pub mod pipistrello_ols;
    pub mod raspberrypi_pico;
    pub mod rdtech_dps;
    pub mod rdtech_tc;
    pub mod rdtech_um;
    pub mod rigol_dg;
    pub mod rigol_ds;
    pub mod rohde_schwarz_sme_0x;
    pub mod saleae_logic16;
    pub mod saleae_logic_pro;
    pub mod scpi_dmm;
    pub mod scpi_pps;
    pub mod serial_dmm;
    pub mod serial_lcr;
    pub mod siglent_sdl10x0;
    pub mod siglent_sds;
    pub mod sysclk_lwla;
    pub mod sysclk_sla5032;
    pub mod teleinfo;
    pub mod testo;
    pub mod tondaj_sl_814;
    pub mod uni_t_dmm;
    pub mod uni_t_ut181a;
    pub mod uni_t_ut32x;
    pub mod yokogawa_dlm;
    pub mod zeroplus_logic_cube;
    pub mod zketech_ebd_usb;
}

/// Registers every driver.
pub fn register_all(registry: &mut core::Registry) {
    registry.register(Box::new(drivers::demo::DemoDriver));
    registry.register(Box::new(drivers::logicport::LogicPortDriver));
    drivers::serial_dmm::register_all(registry);
    registry.register(Box::new(drivers::uni_t_ut32x::Ut32xDriver));
    drivers::serial_lcr::register_all(registry);
    drivers::mic_985xx::register_all(registry);
    registry.register(Box::new(drivers::juntek_jds6600::JdsDriver));
    registry.register(Box::new(drivers::korad_kaxxxxp::KoradDriver));
    registry.register(Box::new(drivers::fluke_45::Fluke45Driver));
    registry.register(Box::new(drivers::fluke_dmm::FlukeDriver));
    registry.register(Box::new(drivers::agilent_dmm::AgilentDmmDriver));
    registry.register(Box::new(drivers::atten_pps3xxx::AttenPpsDriver));
    registry.register(Box::new(drivers::center_3xx::Center309Driver));
    registry.register(Box::new(drivers::center_3xx::VoltcraftK204Driver));
    registry.register(Box::new(drivers::itech_it8500::ItechIt8500Driver));
    registry.register(Box::new(drivers::ipdbg_la::IpdbgLaDriver));
    registry.register(Box::new(drivers::asix_omega_rtm_cli::AsixOmegaRtmCliDriver));
    registry.register(Box::new(drivers::arachnid_labs_re_load_pro::ReloadProDriver));
    registry.register(Box::new(drivers::ols::OlsDriver));
    registry.register(Box::new(drivers::rdtech_um::RdtechUmDriver));
    registry.register(Box::new(drivers::atorch::AtorchDriver));
    registry.register(Box::new(drivers::manson_hcs_3xxx::MansonDriver));
    registry.register(Box::new(drivers::mastech_ms6514::MastechMs6514Driver));
    registry.register(Box::new(drivers::gmc_mh_1x_2x::GmcRs232Driver));
    registry.register(Box::new(drivers::gmc_mh_1x_2x::GmcBd232Driver));
    registry.register(Box::new(drivers::teleinfo::TeleinfoDriver));
    registry.register(Box::new(drivers::gwinstek_gds_800::GwinstekGds800Driver));
    registry.register(Box::new(drivers::gwinstek_gpd::GpdDriver));
    registry.register(Box::new(drivers::bkprecision_1856d::Bkprecision1856dDriver));
    registry.register(Box::new(drivers::motech_lps_30x::MotechLps301Driver));
    registry.register(Box::new(drivers::gwinstek_psp::PspDriver));
    registry.register(Box::new(drivers::appa_55ii::Appa55iiDriver));
    registry.register(Box::new(drivers::devantech_eth008::DevantechEth008Driver));
    registry.register(Box::new(drivers::uni_t_ut181a::UniTUt181aDriver));
    registry.register(Box::new(drivers::link_mso19::LinkMso19Driver));
    registry.register(Box::new(drivers::cem_dt_885x::CemDt885xDriver));
    registry.register(Box::new(drivers::rdtech_dps::RdtechDpsDriver));
    registry.register(Box::new(drivers::rdtech_dps::RdtechRdDriver));
    registry.register(Box::new(drivers::maynuo_m97::MaynuoM97Driver));
    registry.register(Box::new(drivers::norma_dmm::NormaDmmDriver));
    registry.register(Box::new(drivers::norma_dmm::SiemensB102xDriver));
    registry.register(Box::new(drivers::rigol_dg::RigolDgDriver));
    registry.register(Box::new(drivers::hameg_hmo::HamegHmoDriver));
    registry.register(Box::new(drivers::rigol_ds::RigolDsDriver));
    registry.register(Box::new(drivers::lecroy_xstream::LecroyXstreamDriver));
    registry.register(Box::new(drivers::scpi_pps::ScpiPpsDriver));
    registry.register(Box::new(drivers::scpi_pps::HpibPpsDriver));
    registry.register(Box::new(drivers::rohde_schwarz_sme_0x::SmeDriver));
    registry.register(Box::new(drivers::hp_59306a::Hp59306aDriver));
    registry.register(Box::new(drivers::siglent_sdl10x0::SiglentSdl10x0Driver));
    registry.register(Box::new(drivers::siglent_sds::SiglentSdsDriver));
    registry.register(Box::new(drivers::raspberrypi_pico::RaspberryPiPicoDriver));
    registry.register(Box::new(drivers::rdtech_tc::RdtechTcDriver));
    registry.register(Box::new(drivers::dcttech_usbrelay::UsbRelayDriver));
    registry.register(Box::new(drivers::microchip_pickit2::Pickit2Driver));
    registry.register(Box::new(drivers::fx2lafw::Fx2lafwDriver));
    registry.register(Box::new(drivers::saleae_logic16::Logic16Driver));
    registry.register(Box::new(drivers::saleae_logic_pro::LogicProDriver));
    registry.register(Box::new(drivers::scpi_dmm::ScpiDmmDriver));
    registry.register(Box::new(drivers::ftdi_la::FtdiLaDriver));
    registry.register(Box::new(drivers::testo::TestoDriver));
    registry.register(Box::new(drivers::icstation_usbrelay::IcstationUsbrelayDriver));
    registry.register(Box::new(drivers::ikalogic_scanalogic2::Scanalogic2Driver));
    registry.register(Box::new(drivers::ikalogic_scanaplus::ScanaPlusDriver));
    registry.register(Box::new(drivers::asix_sigma::AsixSigmaDriver));
    registry.register(Box::new(drivers::chronovu_la::ChronovuLaDriver));
    registry.register(Box::new(drivers::kecheng_kc_330b::Kc330bDriver));
    registry.register(Box::new(drivers::conrad_digi_35_cpu::ConradDigi35CpuDriver));
    registry.register(Box::new(drivers::kern_scale::KernScaleDriver));
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
    registry.register(Box::new(drivers::colead_slm::ColeadSlmDriver));
    registry.register(Box::new(drivers::tondaj_sl_814::TondajSl814Driver));
    registry.register(Box::new(drivers::beaglelogic::BeagleLogicDriver));
    registry.register(Box::new(drivers::yokogawa_dlm::YokogawaDlmDriver));
    registry.register(Box::new(drivers::pce_322a::Pce322aDriver));
    registry.register(Box::new(drivers::lascar_el_usb::LascarDriver));
    registry.register(Box::new(drivers::sysclk_sla5032::Sla5032Driver));
    registry.register(Box::new(drivers::hp_3457a::Hp3457aDriver));
    // Linux alone has the files it reads, as libsigrok builds it there alone.
    #[cfg(target_os = "linux")]
    registry.register(Box::new(drivers::baylibre_acme::BaylibreAcmeDriver::default()));
    registry.register(Box::new(drivers::hp_3478a::Hp3478aDriver));
}
