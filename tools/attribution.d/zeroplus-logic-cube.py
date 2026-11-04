# The zeroplus-logic-cube device model (emu/src/models/zeroplus-logic-cube.rs); see tools/attribution.py.
MAP["emu/src/models/zeroplus-logic-cube.rs"] = (
    ["libsigrok/src/hardware/zeroplus-logic-cube/api.c (the models, the capture's address arithmetic)",
     "libsigrok/src/hardware/zeroplus-logic-cube/analyzer.c (the registers, the start status, the clock table)",
     "libsigrok/src/hardware/zeroplus-logic-cube/analyzer.h",
     "libsigrok/src/hardware/zeroplus-logic-cube/gl_usb.c (the requests)",
     "libsigrok/src/hardware/zeroplus-logic-cube/protocol.c (the memory sizes)"],
    "a port of the sigrok counterparts: api.c and protocol.c are GPL-3.0-or-later, as this file is; "
    "analyzer.c, analyzer.h and gl_usb.c carry a BSD-style licence, whose notice is at CLOCKS")

# The port of the driver (src/drivers/zeroplus_logic_cube).
MAP["src/drivers/zeroplus_logic_cube/mod.rs"] = (
    ["libsigrok/src/hardware/zeroplus-logic-cube/api.c"], INDEPENDENT)
MAP["src/drivers/zeroplus_logic_cube/protocol.rs"] = (
    ["libsigrok/src/hardware/zeroplus-logic-cube/api.c (the models, the capture's arithmetic)",
     "libsigrok/src/hardware/zeroplus-logic-cube/analyzer.c",
     "libsigrok/src/hardware/zeroplus-logic-cube/analyzer.h",
     "libsigrok/src/hardware/zeroplus-logic-cube/gl_usb.c",
     "libsigrok/src/hardware/zeroplus-logic-cube/protocol.c",
     "libsigrok/src/hardware/zeroplus-logic-cube/protocol.h"],
    "a port of the sigrok counterparts: api.c and protocol.c are GPL-3.0-or-later, as this file is; "
    "analyzer.c, analyzer.h and gl_usb.c carry a BSD-style licence, whose notice is below")
