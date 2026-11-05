# The port (src/drivers/fluke_dmm); see tools/attribution.py.
MAP["src/drivers/fluke_dmm/mod.rs"] = (
    ["libsigrok/src/hardware/fluke-dmm/api.c",
     "libsigrok/src/hardware/fluke-dmm/protocol.c"], INDEPENDENT)
MAP["src/drivers/fluke_dmm/protocol.rs"] = (
    ["libsigrok/src/hardware/fluke-dmm/api.c",
     "libsigrok/src/hardware/fluke-dmm/protocol.c",
     "libsigrok/src/hardware/fluke-dmm/protocol.h",
     "libsigrok/src/hardware/fluke-dmm/fluke-18x.c",
     "libsigrok/src/hardware/fluke-dmm/fluke-190.c",
     "libsigrok/src/hardware/fluke-dmm/fluke-28x.c"], INDEPENDENT)
