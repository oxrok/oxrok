# The port of the driver (src/drivers/ftdi_la) and the libftdi compatible
# FTDI port it drives (src/ftdi/libftdi.rs).
MAP["src/drivers/ftdi_la/mod.rs"] = (
    ["libsigrok/src/hardware/ftdi-la/api.c"], INDEPENDENT)
MAP["src/drivers/ftdi_la/protocol.rs"] = (
    ["libsigrok/src/hardware/ftdi-la/protocol.c", "libsigrok/src/hardware/ftdi-la/protocol.h"],
    INDEPENDENT)
MAP["src/ftdi/libftdi.rs"] = (
    ["libftdi1 (ftdi.c), by Intra2net AG and the libftdi developers"],
    "written for oxrok to send what libftdi 1.5 sends; ftdi_to_clkbits, ftdi_to_clkbits_AM and "
    "ftdi_convert_baudrate are ported (LGPL-2.1-only, taken under its section 3 as "
    "GPL-3.0-or-later)")
