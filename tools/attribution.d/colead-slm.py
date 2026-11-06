# The port (src/drivers/colead_slm); see tools/attribution.py.
MAP["src/drivers/colead_slm/mod.rs"] = (
    ["libsigrok/src/hardware/colead-slm/api.c",
     "libsigrok/src/hardware/colead-slm/protocol.c"], INDEPENDENT)
MAP["src/drivers/colead_slm/protocol.rs"] = (
    ["libsigrok/src/hardware/colead-slm/protocol.c",
     "libsigrok/src/hardware/colead-slm/protocol.h"], INDEPENDENT)
MAP["src/drivers/colead_slm/tests.rs"] = (
    ["libsigrok/src/hardware/colead-slm/protocol.c"], INDEPENDENT)
