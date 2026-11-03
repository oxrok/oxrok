// Counterpart in sigrok:
//   libsigrok/src/hardware/serial-dmm/api.c (the model table),
//   Copyright (C) 2012-2013 Uwe Hermann <uwe@hermann-uwe.de>,
//   Copyright (C) 2012 Alexandru Gagniuc <mr.nuke.me@gmail.com>,
//   with commits by Bert Vermeulen, Gerhard Sittig, Daniel Elstner,
//   Martin Ling, Aurelien Jacobs, Matthias Heidbrink, Janne Huttunen,
//   Frank Stettner, Soeren Apel, Rainer Schmid, Mike McCauley,
//   Alexander Zakharov, Guido Trentalancia, Sebastian Bissbort
//   (libsigrok libsigrok-unreleased-1855-g20baa77f).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it.

//! The supported meters: one entry per driver id, with the chipset
//! protocol and the serial parameters of the cable.

use crate::dmm::lcd::Fs9721Symbols;
use crate::dmm::ranged::Es519xx;
use crate::dmm::{
    Protocol, Reading, ascii, brymen, eev121gw, lcd, mastech, meterman, ranged, segments,
};
use crate::serial::Port;

/// One supported meter.
pub struct Model {
    /// The driver id, e.g. `uni-t-ut61e-ser`.
    pub id: &'static str,
    /// The vendor.
    pub vendor: &'static str,
    /// The model, with the cable when it matters.
    pub model: &'static str,
    /// The port to open when the scan gets no `conn=`: a HID cable
    /// name for the meters that come with one, else empty.
    pub conn: &'static str,
    /// The serial parameters of the cable. Cables on HID adapters that
    /// have no settable line ignore them.
    pub serialcomm: &'static str,
    /// The chipset protocol.
    pub protocol: Protocol,
    /// For meters that must be asked for a packet: how long to wait
    /// for the answer before asking again, 0 for the default.
    pub request_timeout_ms: u64,
    /// For meters that must be asked: how long to wait after a packet
    /// before asking for the next.
    pub request_delay_ms: u64,
    /// Line handling the meter needs right after the port is opened.
    pub after_open: Option<fn(&mut dyn Port) -> crate::core::Result<()>>,
    /// The meter's request ignores whether its write went out, as
    /// `sr_brymen_bm86x_packet_request` does (it always returns
    /// `SR_OK`): a failed write is then no failure. For the other
    /// meters a failed request ends the scan without a device, and an
    /// acquisition at the requests `receive_data` sends.
    pub request_unchecked: bool,
}

macro_rules! single {
    ($f:expr) => {
        |buf: &[u8]| -> Vec<Reading> { $f(buf).into_iter().collect() }
    };
}

const fn protocol(
    packet_size: usize,
    valid: fn(&[u8]) -> bool,
    parse: fn(&[u8]) -> Vec<Reading>,
) -> Protocol {
    Protocol {
        packet_size,
        frame: None,
        channels: 1,
        channel_names: &[],
        valid,
        parse,
        request: None,
    }
}

const fn requested(p: Protocol, request: &'static [u8]) -> Protocol {
    Protocol { request: Some(request), ..p }
}

fn fs9721_none(b: &[u8]) -> Vec<Reading> {
    lcd::fs9721_parse(b, Fs9721Symbols::None).into_iter().collect()
}
fn fs9721_00c(b: &[u8]) -> Vec<Reading> {
    lcd::fs9721_parse(b, Fs9721Symbols::Temp00C).into_iter().collect()
}
fn fs9721_01c(b: &[u8]) -> Vec<Reading> {
    lcd::fs9721_parse(b, Fs9721Symbols::Temp01C).into_iter().collect()
}
fn fs9721_10c(b: &[u8]) -> Vec<Reading> {
    lcd::fs9721_parse(b, Fs9721Symbols::Temp10C).into_iter().collect()
}
fn fs9721_01f10c(b: &[u8]) -> Vec<Reading> {
    lcd::fs9721_parse(b, Fs9721Symbols::Temp01F10C).into_iter().collect()
}
fn fs9721_max_c_min(b: &[u8]) -> Vec<Reading> {
    lcd::fs9721_parse(b, Fs9721Symbols::MaxCMin).into_iter().collect()
}
fn fs9922(b: &[u8]) -> Vec<Reading> {
    lcd::fs9922_parse(b, false).into_iter().collect()
}
fn fs9922_z1_diode(b: &[u8]) -> Vec<Reading> {
    lcd::fs9922_parse(b, true).into_iter().collect()
}
fn es_2400_11b_valid(b: &[u8]) -> bool {
    ranged::es519xx_valid(b, Es519xx::Baud2400x11)
}
fn es_2400_11b(b: &[u8]) -> Vec<Reading> {
    ranged::es519xx_parse(b, Es519xx::Baud2400x11).into_iter().collect()
}
fn es_19200_11b_valid(b: &[u8]) -> bool {
    ranged::es519xx_valid(b, Es519xx::Baud19200x11)
}
fn es_19200_11b(b: &[u8]) -> Vec<Reading> {
    ranged::es519xx_parse(b, Es519xx::Baud19200x11).into_iter().collect()
}
fn es_19200_14b_valid(b: &[u8]) -> bool {
    ranged::es519xx_valid(b, Es519xx::Baud19200x14)
}
fn es_19200_14b(b: &[u8]) -> Vec<Reading> {
    ranged::es519xx_parse(b, Es519xx::Baud19200x14).into_iter().collect()
}

const FS9721: Protocol = protocol(lcd::FS9721_SIZE, lcd::fs9721_valid, fs9721_none);
const FS9922: Protocol = protocol(lcd::FS9922_SIZE, lcd::fs9922_valid, fs9922);
const DTM0660: Protocol =
    protocol(lcd::DTM0660_SIZE, lcd::dtm0660_valid, single!(lcd::dtm0660_parse));
const METEX14: Protocol = requested(
    protocol(ascii::METEX14_SIZE, ascii::metex14_valid, single!(ascii::metex14_parse)),
    ascii::METEX14_REQUEST,
);
const METEX14_X4: Protocol = Protocol {
    packet_size: 4 * ascii::METEX14_SIZE,
    frame: None,
    channels: 4,
    channel_names: &[],
    valid: ascii::metex14_x4_valid,
    parse: ascii::metex14_x4_parse,
    request: Some(ascii::METEX14_REQUEST),
};
const UT71X: Protocol =
    protocol(ranged::UT71X_SIZE, ranged::ut71x_valid, single!(ranged::ut71x_parse));
const ES519XX_2400_11B: Protocol =
    protocol(ranged::ES519XX_11B_SIZE, es_2400_11b_valid, es_2400_11b);
const ES519XX_19200_11B: Protocol =
    protocol(ranged::ES519XX_11B_SIZE, es_19200_11b_valid, es_19200_11b);
const ES519XX_19200_14B: Protocol =
    protocol(ranged::ES519XX_14B_SIZE, es_19200_14b_valid, es_19200_14b);
const VC870: Protocol =
    protocol(ranged::VC870_SIZE, ranged::vc870_valid, single!(ranged::vc870_parse));
const VC96: Protocol = protocol(ascii::VC96_SIZE, ascii::vc96_valid, single!(ascii::vc96_parse));
const ASYCII: Protocol =
    protocol(ascii::ASYCII_SIZE, ascii::asycii_valid, single!(ascii::asycii_parse));
const M2110: Protocol =
    protocol(ascii::M2110_SIZE, ascii::m2110_valid, single!(ascii::m2110_parse));
const RS9LCD: Protocol =
    protocol(segments::RS9LCD_SIZE, segments::rs9lcd_valid, single!(segments::rs9lcd_parse));
const BM25X: Protocol =
    protocol(segments::BM25X_SIZE, segments::bm25x_valid, single!(segments::bm25x_parse));
const QM1578: Protocol =
    protocol(segments::QM1578_SIZE, segments::qm1578_valid, single!(segments::qm1578_parse));
const UT372: Protocol =
    protocol(segments::UT372_SIZE, segments::ut372_valid, single!(segments::ut372_parse));
const BM52X: Protocol = Protocol {
    packet_size: brymen::BRYMEN_SIZE,
    frame: None,
    channels: brymen::BRYMEN_DISPLAYS,
    channel_names: &[],
    valid: brymen::bm52x_valid,
    parse: brymen::bm52x_parse,
    request: Some(brymen::BM52X_REQUEST),
};
const BM82X: Protocol =
    Protocol { valid: brymen::bm82x_valid, request: Some(brymen::BM82X_REQUEST), ..BM52X };
fn bm85x_never(_: &[u8]) -> bool {
    false
}
const BM85X: Protocol = Protocol {
    packet_size: brymen::BM85X_MIN_SIZE,
    frame: Some(brymen::bm85x_frame),
    channels: 1,
    channel_names: &[],
    valid: bm85x_never,
    parse: single!(brymen::bm85x_parse),
    request: Some(brymen::BM85X_REQUEST),
};
const EEV121GW: Protocol = Protocol {
    packet_size: eev121gw::EEV121GW_SIZE,
    frame: None,
    channels: eev121gw::EEV121GW_DISPLAYS,
    channel_names: &eev121gw::EEV121GW_CHANNELS,
    valid: eev121gw::eev121gw_valid,
    parse: eev121gw::eev121gw_parse,
    request: None,
};
const MS2115B: Protocol = Protocol {
    packet_size: mastech::MS2115B_SIZE,
    frame: None,
    channels: mastech::MS2115B_DISPLAYS,
    channel_names: &mastech::MS2115B_CHANNELS,
    valid: mastech::ms2115b_valid,
    parse: mastech::ms2115b_parse,
    request: None,
};
const MS8250D: Protocol =
    protocol(mastech::MS8250D_SIZE, mastech::ms8250d_valid, single!(mastech::ms8250d_parse));
const MM38XR: Protocol =
    protocol(meterman::MM38XR_SIZE, meterman::mm38xr_valid, single!(meterman::mm38xr_parse));
const BM86X: Protocol = Protocol {
    valid: brymen::bm86x_valid,
    parse: brymen::bm86x_parse,
    request: Some(brymen::BM86X_REQUEST),
    ..BM52X
};

const fn m(
    id: &'static str,
    vendor: &'static str,
    model: &'static str,
    serialcomm: &'static str,
    protocol: Protocol,
) -> Model {
    Model {
        id,
        vendor,
        model,
        conn: "",
        serialcomm,
        protocol,
        request_timeout_ms: 0,
        request_delay_ms: 0,
        after_open: None,
        request_unchecked: false,
    }
}

/// The BM85x wants an RTS pulse before it talks: wait, RTS high, wait,
/// RTS low, wait, RTS high, wait, 100 ms each.
#[allow(clippy::doc_markdown)]
fn bm85x_after_open(port: &mut dyn Port) -> crate::core::Result<()> {
    let pause = || std::thread::sleep(std::time::Duration::from_millis(100));
    pause();
    port.set_handshake(Some(true), None)?;
    pause();
    port.set_handshake(Some(false), None)?;
    pause();
    port.set_handshake(Some(true), None)?;
    pause();
    Ok(())
}

const fn with_after_open(
    mut model: Model,
    f: fn(&mut dyn Port) -> crate::core::Result<()>,
) -> Model {
    model.after_open = Some(f);
    model
}

const fn timed(mut model: Model, timeout_ms: u64, delay_ms: u64) -> Model {
    model.request_timeout_ms = timeout_ms;
    model.request_delay_ms = delay_ms;
    model
}

const fn unchecked_request(mut model: Model) -> Model {
    model.request_unchecked = true;
    model
}

const fn on_hid(mut model: Model, conn: &'static str) -> Model {
    model.conn = conn;
    model
}

const fn with_parse(mut p: Protocol, parse: fn(&[u8]) -> Vec<Reading>) -> Protocol {
    p.parse = parse;
    p
}

/// All supported meters, sorted by chipset then model as libsigrok
/// lists them.
pub static MODELS: &[Model] = &[
    m("digitech-qm1578", "Digitech", "QM1578", "115200/8n1", QM1578),
    m("mastech-ms2115b", "MASTECH", "MS2115B", "1200/8n1", MS2115B),
    m("mastech-ms8250d", "MASTECH", "MS8250D", "2400/8n1/rts=0/dtr=1", MS8250D),
    m("meterman-38xr", "Meterman", "38XR", "9600/8n1/rts=0/dtr=1", MM38XR),
    m("metrix-mx56c", "Metrix", "MX56C", "2400/8n1", ASYCII),
    m("brymen-bm25x", "Brymen", "BM25x", "9600/8n1/rts=1/dtr=1", BM25X),
    on_hid(timed(m("brymen-bm52x", "Brymen", "BM52x", "9600/8n1", BM52X), 4000, 500), "hid/bu86x"),
    on_hid(timed(m("brymen-bm82x", "Brymen", "BM82x", "9600/8n1", BM82X), 4000, 500), "hid/bu86x"),
    with_after_open(
        timed(m("brymen-bm85x", "Brymen", "BM85x", "9600/8n1/dtr=1/rts=1", BM85X), 2000, 400),
        bm85x_after_open,
    ),
    on_hid(
        unchecked_request(timed(m("brymen-bm86x", "Brymen", "BM86x", "9600/8n1", BM86X), 500, 100)),
        "hid/bu86x",
    ),
    m("peaktech-3415", "PeakTech", "3415", "2400/8n1/rts=0/dtr=1", DTM0660),
    m("velleman-dvm4100", "Velleman", "DVM4100", "2400/8n1/rts=0/dtr=1", DTM0660),
    m("eevblog-121gw", "EEVblog", "121GW", "115200/8n1", EEV121GW),
    m("iso-tech-idm103n", "ISO-TECH", "IDM103N", "2400/7o1/rts=0/dtr=1", ES519XX_2400_11B),
    m(
        "tenma-72-7750-ser",
        "Tenma",
        "72-7750 (UT-D02 cable)",
        "19200/7o1/rts=0/dtr=1",
        ES519XX_19200_11B,
    ),
    m(
        "uni-t-ut60g-ser",
        "UNI-T",
        "UT60G (UT-D02 cable)",
        "19200/7o1/rts=0/dtr=1",
        ES519XX_19200_11B,
    ),
    m(
        "uni-t-ut61e-ser",
        "UNI-T",
        "UT61E (UT-D02 cable)",
        "19200/7o1/rts=0/dtr=1",
        ES519XX_19200_14B,
    ),
    m("digitek-dt4000zc", "Digitek", "DT4000ZC", "2400/8n1/dtr=1", with_parse(FS9721, fs9721_10c)),
    m("mastech-ms8250b", "MASTECH", "MS8250B", "2400/8n1/rts=0/dtr=1", FS9721),
    m("pce-pce-dm32", "PCE", "PCE-DM32", "2400/8n1", with_parse(FS9721, fs9721_01f10c)),
    m("peaktech-3330", "PeakTech", "3330", "2400/8n1/dtr=1", with_parse(FS9721, fs9721_01f10c)),
    m(
        "tecpel-dmm-8061-ser",
        "Tecpel",
        "DMM-8061 (UT-D02 cable)",
        "2400/8n1/rts=0/dtr=1",
        with_parse(FS9721, fs9721_00c),
    ),
    m(
        "tekpower-tp4000ZC",
        "TekPower",
        "TP4000ZC",
        "2400/8n1/dtr=1",
        with_parse(FS9721, fs9721_10c),
    ),
    m(
        "tenma-72-7745-ser",
        "Tenma",
        "72-7745 (UT-D02 cable)",
        "2400/8n1/rts=0/dtr=1",
        with_parse(FS9721, fs9721_00c),
    ),
    m("uni-t-ut60a-ser", "UNI-T", "UT60A (UT-D02 cable)", "2400/8n1/rts=0/dtr=1", FS9721),
    m(
        "uni-t-ut60e-ser",
        "UNI-T",
        "UT60E (UT-D02 cable)",
        "2400/8n1/rts=0/dtr=1",
        with_parse(FS9721, fs9721_00c),
    ),
    m("va-va18b", "V&A", "VA18B", "2400/8n1", with_parse(FS9721, fs9721_01c)),
    m("va-va40b", "V&A", "VA40B", "2400/8n1", with_parse(FS9721, fs9721_max_c_min)),
    m("voltcraft-vc820-ser", "Voltcraft", "VC-820 (UT-D02 cable)", "2400/8n1/rts=0/dtr=1", FS9721),
    m(
        "voltcraft-vc840-ser",
        "Voltcraft",
        "VC-840 (UT-D02 cable)",
        "2400/8n1/rts=0/dtr=1",
        with_parse(FS9721, fs9721_00c),
    ),
    m("gwinstek-gdm-397", "GW Instek", "GDM-397", "2400/8n1/rts=0/dtr=1", FS9922),
    m("peaktech-2025", "PeakTech", "2025", "2400/8n1/rts=0/dtr=1", FS9922),
    m("sparkfun-70c", "SparkFun", "70C", "2400/8n1/rts=0/dtr=1", FS9922),
    m("uni-t-ut61b-ser", "UNI-T", "UT61B (UT-D02 cable)", "2400/8n1/rts=0/dtr=1", FS9922),
    m("uni-t-ut61c-ser", "UNI-T", "UT61C (UT-D02 cable)", "2400/8n1/rts=0/dtr=1", FS9922),
    m("uni-t-ut61d-ser", "UNI-T", "UT61D (UT-D02 cable)", "2400/8n1/rts=0/dtr=1", FS9922),
    on_hid(m("victor-dmm", "Victor", "Victor DMMs", "2400/8n1", FS9922), "hid/victor"),
    m(
        "voltcraft-vc830-ser",
        "Voltcraft",
        "VC-830 (UT-D02 cable)",
        "2400/8n1/rts=0/dtr=1",
        with_parse(FS9922, fs9922_z1_diode),
    ),
    m("bbcgm-2010", "BBC Goertz Metrawatt", "M2110", "1200/7n2", M2110),
    m("mastech-mas345", "MASTECH", "MAS345", "600/7n2/rts=0/dtr=1", METEX14),
    m("metex-m3640d", "Metex", "M-3640D", "1200/7n2/rts=0/dtr=1", METEX14),
    m("metex-m3860m", "Metex", "M-3860M", "9600/7n2/rts=0/dtr=1", METEX14_X4),
    m("metex-m4650cr", "Metex", "M-4650CR", "1200/7n2/rts=0/dtr=1", METEX14),
    m("metex-me21", "Metex", "ME-21", "2400/7n2/rts=0/dtr=1", METEX14),
    m("metex-me31", "Metex", "ME-31", "600/7n2/rts=0/dtr=1", METEX14),
    m("peaktech-3410", "PeakTech", "3410", "600/7n2/rts=0/dtr=1", METEX14),
    m("peaktech-4370", "PeakTech", "4370", "1200/7n2/rts=0/dtr=1", METEX14),
    m("peaktech-4390a", "PeakTech", "4390A", "9600/7n2/rts=0/dtr=1", METEX14_X4),
    m("radioshack-22-168", "RadioShack", "22-168", "1200/7n2/rts=0/dtr=1", METEX14),
    m("radioshack-22-805", "RadioShack", "22-805", "600/7n2/rts=0/dtr=1", METEX14),
    timed(
        m("voltcraft-m3650cr", "Voltcraft", "M-3650CR", "1200/7n2/rts=0/dtr=1", METEX14),
        150,
        20,
    ),
    m("voltcraft-m3650d", "Voltcraft", "M-3650D", "1200/7n2/rts=0/dtr=1", METEX14),
    m("voltcraft-m4650cr", "Voltcraft", "M-4650CR", "1200/7n2/rts=0/dtr=1", METEX14),
    timed(m("voltcraft-me42", "Voltcraft", "ME-42", "600/7n2/rts=0/dtr=1", METEX14), 250, 60),
    m("radioshack-22-812", "RadioShack", "22-812", "4800/8n1/rts=0/dtr=1", RS9LCD),
    m("tenma-72-7730-ser", "Tenma", "72-7730 (UT-D02 cable)", "2400/7o1/rts=0/dtr=1", UT71X),
    m("tenma-72-7732-ser", "Tenma", "72-7732 (UT-D02 cable)", "2400/7o1/rts=0/dtr=1", UT71X),
    m("tenma-72-9380a-ser", "Tenma", "72-9380A (UT-D02 cable)", "2400/7o1/rts=0/dtr=1", UT71X),
    m("uni-t-ut71a-ser", "UNI-T", "UT71A (UT-D02 cable)", "2400/7o1/rts=0/dtr=1", UT71X),
    m("uni-t-ut71b-ser", "UNI-T", "UT71B (UT-D02 cable)", "2400/7o1/rts=0/dtr=1", UT71X),
    m("uni-t-ut71c-ser", "UNI-T", "UT71C (UT-D02 cable)", "2400/7o1/rts=0/dtr=1", UT71X),
    m("uni-t-ut71d-ser", "UNI-T", "UT71D (UT-D02 cable)", "2400/7o1/rts=0/dtr=1", UT71X),
    m("uni-t-ut71e-ser", "UNI-T", "UT71E (UT-D02 cable)", "2400/7o1/rts=0/dtr=1", UT71X),
    m("uni-t-ut804-ser", "UNI-T", "UT804", "2400/7o1/rts=0/dtr=1", UT71X),
    m("voltcraft-vc920-ser", "Voltcraft", "VC-920 (UT-D02 cable)", "2400/7o1/rts=0/dtr=1", UT71X),
    m("voltcraft-vc940-ser", "Voltcraft", "VC-940 (UT-D02 cable)", "2400/7o1/rts=0/dtr=1", UT71X),
    m("voltcraft-vc960-ser", "Voltcraft", "VC-960 (UT-D02 cable)", "2400/7o1/rts=0/dtr=1", UT71X),
    m("voltcraft-vc870-ser", "Voltcraft", "VC-870 (UT-D02 cable)", "9600/8n1/rts=0/dtr=1", VC870),
    m("voltcraft-vc96", "Voltcraft", "VC-96", "1200/8n2", VC96),
];

/// The meters on the UNI-T UT-D04 cable, a CH9325 HID bridge: the ids
/// of libsigrok's uni-t-dmm driver, each the HID twin of a serial
/// entry (the same chipset at the same line parameters).
const HID_MODELS: &[(&str, &str, &str, &str)] = &[
    ("tenma-72-7750", "Tenma", "72-7750", "tenma-72-7750-ser"),
    ("uni-t-ut60g", "UNI-T", "UT60G", "uni-t-ut60g-ser"),
    ("uni-t-ut61e", "UNI-T", "UT61E", "uni-t-ut61e-ser"),
    ("tecpel-dmm-8061", "Tecpel", "DMM-8061", "tecpel-dmm-8061-ser"),
    ("tenma-72-7745", "Tenma", "72-7745", "tenma-72-7745-ser"),
    ("uni-t-ut60a", "UNI-T", "UT60A", "uni-t-ut60a-ser"),
    ("uni-t-ut60e", "UNI-T", "UT60E", "uni-t-ut60e-ser"),
    ("voltcraft-vc820", "Voltcraft", "VC-820", "voltcraft-vc820-ser"),
    ("voltcraft-vc840", "Voltcraft", "VC-840", "voltcraft-vc840-ser"),
    ("uni-t-ut61b", "UNI-T", "UT61B", "uni-t-ut61b-ser"),
    ("uni-t-ut61c", "UNI-T", "UT61C", "uni-t-ut61c-ser"),
    ("uni-t-ut61d", "UNI-T", "UT61D", "uni-t-ut61d-ser"),
    ("voltcraft-vc830", "Voltcraft", "VC-830", "voltcraft-vc830-ser"),
    ("uni-t-ut372", "UNI-T", "UT372", ""),
    ("tenma-72-7730", "Tenma", "72-7730", "tenma-72-7730-ser"),
    ("tenma-72-7732", "Tenma", "72-7732", "tenma-72-7732-ser"),
    ("tenma-72-9380a", "Tenma", "72-9380A", "tenma-72-9380a-ser"),
    ("uni-t-ut71a", "UNI-T", "UT71A", "uni-t-ut71a-ser"),
    ("uni-t-ut71b", "UNI-T", "UT71B", "uni-t-ut71b-ser"),
    ("uni-t-ut71c", "UNI-T", "UT71C", "uni-t-ut71c-ser"),
    ("uni-t-ut71d", "UNI-T", "UT71D", "uni-t-ut71d-ser"),
    ("uni-t-ut71e", "UNI-T", "UT71E", "uni-t-ut71e-ser"),
    ("uni-t-ut804", "UNI-T", "UT804", "uni-t-ut804-ser"),
    ("voltcraft-vc920", "Voltcraft", "VC-920", "voltcraft-vc920-ser"),
    ("voltcraft-vc940", "Voltcraft", "VC-940", "voltcraft-vc940-ser"),
    ("voltcraft-vc960", "Voltcraft", "VC-960", "voltcraft-vc960-ser"),
    ("voltcraft-vc870", "Voltcraft", "VC-870", "voltcraft-vc870-ser"),
];

/// The cable of the HID twins.
pub const UT_D04_CONN: &str = "hid/ch9325";

/// The meters on the UT-D04 HID cable, built once from `HID_MODELS`.
pub fn hid_models() -> &'static [Model] {
    static TABLE: std::sync::OnceLock<Vec<Model>> = std::sync::OnceLock::new();
    TABLE.get_or_init(|| {
        HID_MODELS
            .iter()
            .map(|(id, vendor, model, base)| {
                let (serialcomm, protocol) = if base.is_empty() {
                    ("2400/8n1", UT372)
                } else {
                    let b = find_serial(base).expect("HID twin has a serial base");
                    (b.serialcomm, b.protocol)
                };
                Model {
                    id,
                    vendor,
                    model,
                    conn: UT_D04_CONN,
                    serialcomm,
                    protocol,
                    request_timeout_ms: 0,
                    request_delay_ms: 0,
                    after_open: None,
                    request_unchecked: false,
                }
            })
            .collect()
    })
}

fn find_serial(id: &str) -> Option<&'static Model> {
    MODELS.iter().find(|m| m.id == id)
}

/// The model with driver id `id`, serial or HID cable.
pub fn find(id: &str) -> Option<&'static Model> {
    find_serial(id).or_else(|| hid_models().iter().find(|m| m.id == id))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ids_are_unique_and_parameters_parse() {
        let mut ids: Vec<&str> = MODELS.iter().map(|m| m.id).collect();
        ids.sort_unstable();
        ids.dedup();
        assert_eq!(ids.len(), MODELS.len());
        for m in MODELS {
            assert!(crate::serial::Params::parse(m.serialcomm).is_ok(), "{}", m.id);
            assert!(m.protocol.packet_size > 0);
        }
        assert_eq!(find("uni-t-ut61e-ser").unwrap().protocol.packet_size, 14);
        assert_eq!(find("metex-m3860m").unwrap().protocol.channels, 4);
        assert_eq!(find("brymen-bm86x").unwrap().conn, "hid/bu86x");
        assert_eq!(find("victor-dmm").unwrap().protocol.packet_size, 14);
        assert_eq!(MODELS.len(), 69);
        assert_eq!(hid_models().len(), 27);
        let e = find("uni-t-ut61e").unwrap();
        assert_eq!(
            (e.conn, e.serialcomm, e.protocol.packet_size),
            (UT_D04_CONN, "19200/7o1/rts=0/dtr=1", 14)
        );
        assert_eq!(find("uni-t-ut372").unwrap().protocol.packet_size, 27);
        assert!(find("brymen-bm85x").unwrap().protocol.frame.is_some());
    }
}
