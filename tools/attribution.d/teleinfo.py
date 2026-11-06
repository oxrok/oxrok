# The port (src/drivers/teleinfo); see tools/attribution.py.
MAP["src/drivers/teleinfo/mod.rs"] = (
    ["libsigrok/src/hardware/teleinfo/api.c",
     "libsigrok/src/hardware/teleinfo/protocol.c"], INDEPENDENT)
MAP["src/drivers/teleinfo/protocol.rs"] = (
    ["libsigrok/src/hardware/teleinfo/protocol.c",
     "libsigrok/src/hardware/teleinfo/protocol.h"], INDEPENDENT)
MAP["src/drivers/teleinfo/tests.rs"] = (
    ["libsigrok/src/hardware/teleinfo/api.c",
     "libsigrok/src/hardware/teleinfo/protocol.c"], INDEPENDENT)
