# The ftdi-la device model (emu/src/models/ftdi-la.rs) and the FTDI shell
# (emu/src/shells/ftdi.rs); see tools/attribution.py.
MAP["emu/src/models/ftdi-la.rs"] = (
    ["libsigrok/src/hardware/ftdi-la/api.c (the chips and the sequence)",
     "libsigrok/src/hardware/ftdi-la/protocol.c (the sample rate and the reads)"], INDEPENDENT)
MAP["emu/src/shells/ftdi.rs"] = (
    ["libftdi1 (ftdi.c: ftdi_to_clkbits, ftdi_convert_baudrate), by Intra2net AG and the "
     "libftdi developers"],
    "written for oxrok from FTDI's application notes; libftdi was the reference for which "
    "requests a driver sends, and libftdi_divisor is a port of libftdi's ftdi_to_clkbits and "
    "ftdi_convert_baudrate (LGPL-2.1-only, taken under its section 3 as GPL-3.0-or-later)")
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
