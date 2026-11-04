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
