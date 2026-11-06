# The port (src/drivers/atorch); see tools/attribution.py.
MAP["src/drivers/atorch/mod.rs"] = (
    ["libsigrok/src/hardware/atorch/api.c",
     "libsigrok/src/hardware/atorch/protocol.c"], INDEPENDENT)
MAP["src/drivers/atorch/protocol.rs"] = (
    ["libsigrok/src/hardware/atorch/protocol.c",
     "libsigrok/src/hardware/atorch/protocol.h"], INDEPENDENT)
MAP["src/drivers/atorch/tests.rs"] = (
    ["libsigrok/src/hardware/atorch/protocol.c"], INDEPENDENT)
