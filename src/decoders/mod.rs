//! Native protocol decoders. Each one reproduces the annotation
//! classes, rows and texts of the libsigrokdecode decoder of the same
//! id, so `-A` selections and existing scripts keep working, and is
//! written from the protocol itself.

#![forbid(unsafe_code)]

pub mod avr_isp;
pub mod can;
pub mod cec;
pub mod counter;
pub mod dali;
pub mod dcf77;
pub mod dmx512;
pub mod eeprom24xx;
pub mod em4100;
pub mod em4305;
pub mod gpib;
pub mod graycode;
pub mod guess_bitrate;
pub mod i2c;
pub mod i2cdemux;
pub mod i2cfilter;
pub mod i2s;
pub mod ieee488;
pub mod ir_nec;
pub mod ir_rc5;
pub mod ir_rc6;
pub mod jitter;
pub mod jtag;
pub mod lin;
pub mod lpc;
pub mod mdio;
pub mod midi;
pub mod miller;
pub mod modbus;
pub mod morse;
pub mod onewire_link;
pub mod parallel;
pub mod ps2;
pub mod pwm;
pub mod rc_encode;
pub mod rgb_led_ws281x;
pub mod sda2506;
pub mod sdcard_sd;
pub mod seven_segment;
pub mod spdif;
pub mod spi;
pub mod spiflash;
pub mod st7735;
pub mod stepper_motor;
pub mod swd;
pub mod tdm_audio;
pub mod timing;
pub mod uart;
pub mod usb_power_delivery;
pub mod usb_signalling;
pub mod wiegand;
pub mod z80;

use crate::decode::DecoderDef;

/// Every decoder in this crate.
pub static ALL: [&DecoderDef; 52] = [
    &avr_isp::DEF,
    &can::DEF,
    &cec::DEF,
    &counter::DEF,
    &dali::DEF,
    &dcf77::DEF,
    &dmx512::DEF,
    &eeprom24xx::DEF,
    &em4100::DEF,
    &em4305::DEF,
    &gpib::DEF,
    &graycode::DEF,
    &guess_bitrate::DEF,
    &i2c::DEF,
    &i2cdemux::DEF,
    &i2cfilter::DEF,
    &i2s::DEF,
    &ieee488::DEF,
    &ir_nec::DEF,
    &ir_rc5::DEF,
    &ir_rc6::DEF,
    &jitter::DEF,
    &jtag::DEF,
    &lin::DEF,
    &lpc::DEF,
    &mdio::DEF,
    &midi::DEF,
    &miller::DEF,
    &modbus::DEF,
    &morse::DEF,
    &onewire_link::DEF,
    &parallel::DEF,
    &ps2::DEF,
    &pwm::DEF,
    &rc_encode::DEF,
    &rgb_led_ws281x::DEF,
    &sda2506::DEF,
    &sdcard_sd::DEF,
    &seven_segment::DEF,
    &spdif::DEF,
    &spi::DEF,
    &spiflash::DEF,
    &st7735::DEF,
    &stepper_motor::DEF,
    &swd::DEF,
    &tdm_audio::DEF,
    &timing::DEF,
    &uart::DEF,
    &usb_power_delivery::DEF,
    &usb_signalling::DEF,
    &wiegand::DEF,
    &z80::DEF,
];
