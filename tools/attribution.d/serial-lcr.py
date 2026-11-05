# The port (src/drivers/serial_lcr, src/lcr); see tools/attribution.py.
MAP["src/drivers/serial_lcr/mod.rs"] = (
    ["libsigrok/src/hardware/serial-lcr/api.c",
     "libsigrok/src/hardware/serial-lcr/protocol.c",
     "libsigrok/src/hardware/serial-lcr/protocol.h"], INDEPENDENT)
MAP["src/drivers/serial_lcr/tests.rs"] = (
    ["libsigrok/src/hardware/serial-lcr/api.c",
     "libsigrok/src/hardware/serial-lcr/protocol.c"], INDEPENDENT)
MAP["src/lcr/mod.rs"] = (
    ["libsigrok/src/hardware/serial-lcr/protocol.h"], INDEPENDENT)
MAP["src/lcr/es51919.rs"] = (["libsigrok/src/lcr/es51919.c"], INDEPENDENT)
MAP["src/lcr/vc4080.rs"] = (["libsigrok/src/lcr/vc4080.c"], INDEPENDENT)
