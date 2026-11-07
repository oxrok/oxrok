//! Native protocol decoders. Each one reproduces the annotation
//! classes, rows and texts of the libsigrokdecode decoder of the same
//! id, so `-A` selections and existing scripts keep working, and is
//! written from the protocol itself.

#![forbid(unsafe_code)]

pub mod ac97;
pub mod am230x;
pub mod atsha204a;
pub mod aud;
pub mod avr_isp;
pub mod avr_pdi;
pub mod can;
pub mod cec;
pub mod counter;
pub mod dali;
pub mod dcf77;
pub mod dmx512;
pub mod ds1307;
pub mod ds2408;
pub mod ds243x;
pub mod ds28ea00;
pub mod dsi;
pub mod edid;
pub mod eeprom24xx;
pub mod em4100;
pub mod em4305;
pub mod flexray;
pub mod gpib;
pub mod graycode;
pub mod guess_bitrate;
pub mod hdcp;
pub mod i2c;
pub mod i2cdemux;
pub mod i2cfilter;
pub mod i2s;
pub mod iec;
pub mod ieee488;
pub mod ir_nec;
pub mod ir_rc5;
pub mod ir_rc6;
pub mod jitter;
pub mod jtag;
pub mod jtag_ejtag;
pub mod jtag_stm32;
pub mod lin;
pub mod lm75;
pub mod lpc;
pub mod maple_bus;
pub mod max7219;
pub mod mcs48;
pub mod mdio;
pub mod microwire;
pub mod midi;
pub mod miller;
pub mod mlx90614;
pub mod modbus;
pub mod morse;
pub mod mrf24j40;
pub mod mxc6225xu;
pub mod nes_gamepad;
pub mod nrf24l01;
pub mod nunchuk;
pub mod onewire_link;
pub mod onewire_network;
pub mod ook;
pub mod parallel;
pub mod pca9571;
pub mod ps2;
pub mod pwm;
pub mod qi;
pub mod rc_encode;
pub mod rfm12;
pub mod rgb_led_spi;
pub mod rgb_led_ws281x;
pub mod rtc8564;
pub mod sda2506;
pub mod sdcard_sd;
pub mod seven_segment;
pub mod signature;
pub mod spdif;
pub mod spi;
pub mod spiflash;
pub mod ssi32;
pub mod st7735;
pub mod stepper_motor;
pub mod swd;
pub mod swim;
pub mod t55xx;
pub mod tca6408a;
pub mod tdm_audio;
pub mod timing;
pub mod tlc5620;
pub mod uart;
pub mod usb_power_delivery;
pub mod usb_signalling;
pub mod wiegand;
pub mod x2444m;
pub mod xfp;
pub mod z80;

use crate::decode::DecoderDef;

/// Every decoder in this crate.
pub static ALL: [&DecoderDef; 94] = [
    &ac97::DEF,
    &am230x::DEF,
    &atsha204a::DEF,
    &aud::DEF,
    &avr_isp::DEF,
    &avr_pdi::DEF,
    &can::DEF,
    &cec::DEF,
    &counter::DEF,
    &dali::DEF,
    &dcf77::DEF,
    &dmx512::DEF,
    &ds1307::DEF,
    &ds2408::DEF,
    &ds243x::DEF,
    &ds28ea00::DEF,
    &dsi::DEF,
    &edid::DEF,
    &eeprom24xx::DEF,
    &em4100::DEF,
    &em4305::DEF,
    &flexray::DEF,
    &gpib::DEF,
    &graycode::DEF,
    &guess_bitrate::DEF,
    &hdcp::DEF,
    &i2c::DEF,
    &i2cdemux::DEF,
    &i2cfilter::DEF,
    &i2s::DEF,
    &iec::DEF,
    &ieee488::DEF,
    &ir_nec::DEF,
    &ir_rc5::DEF,
    &ir_rc6::DEF,
    &jitter::DEF,
    &jtag::DEF,
    &jtag_ejtag::DEF,
    &jtag_stm32::DEF,
    &lin::DEF,
    &lm75::DEF,
    &lpc::DEF,
    &maple_bus::DEF,
    &max7219::DEF,
    &mcs48::DEF,
    &mdio::DEF,
    &microwire::DEF,
    &midi::DEF,
    &miller::DEF,
    &mlx90614::DEF,
    &modbus::DEF,
    &morse::DEF,
    &mrf24j40::DEF,
    &mxc6225xu::DEF,
    &nes_gamepad::DEF,
    &nrf24l01::DEF,
    &nunchuk::DEF,
    &onewire_link::DEF,
    &onewire_network::DEF,
    &ook::DEF,
    &parallel::DEF,
    &pca9571::DEF,
    &ps2::DEF,
    &pwm::DEF,
    &qi::DEF,
    &rc_encode::DEF,
    &rfm12::DEF,
    &rgb_led_spi::DEF,
    &rgb_led_ws281x::DEF,
    &rtc8564::DEF,
    &sda2506::DEF,
    &sdcard_sd::DEF,
    &seven_segment::DEF,
    &signature::DEF,
    &spdif::DEF,
    &spi::DEF,
    &spiflash::DEF,
    &ssi32::DEF,
    &st7735::DEF,
    &stepper_motor::DEF,
    &swd::DEF,
    &swim::DEF,
    &t55xx::DEF,
    &tca6408a::DEF,
    &tdm_audio::DEF,
    &timing::DEF,
    &tlc5620::DEF,
    &uart::DEF,
    &usb_power_delivery::DEF,
    &usb_signalling::DEF,
    &wiegand::DEF,
    &x2444m::DEF,
    &xfp::DEF,
    &z80::DEF,
];
