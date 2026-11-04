# The uni-t-dmm device model (emu/src/models/uni-t-dmm.rs); see tools/attribution.py.
MAP["emu/src/models/uni-t-dmm.rs"] = (
    ["libsigrok/src/hardware/uni-t-dmm/protocol.c (the cable's view of the driver)",
     "libsigrok/src/hardware/uni-t-dmm/api.c"], INDEPENDENT)

# The port of the driver (src/drivers/uni_t_dmm).
MAP["src/drivers/uni_t_dmm/mod.rs"] = (
    ["libsigrok/src/hardware/uni-t-dmm/api.c",
     "libsigrok/src/hardware/uni-t-dmm/protocol.c",
     "libsigrok/src/hardware/uni-t-dmm/protocol.h"], INDEPENDENT)
