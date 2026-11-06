# The port (src/drivers/hameg_hmo); see tools/attribution.py.
MAP["src/drivers/hameg_hmo/mod.rs"] = (
    ["libsigrok/src/hardware/hameg-hmo/api.c",
     "libsigrok/src/hardware/hameg-hmo/protocol.c",
     "libsigrok/src/hardware/hameg-hmo/protocol.h"], INDEPENDENT)
MAP["src/drivers/hameg_hmo/models.rs"] = (
    ["libsigrok/src/hardware/hameg-hmo/protocol.c"], INDEPENDENT)
MAP["src/drivers/hameg_hmo/protocol.rs"] = (
    ["libsigrok/src/hardware/hameg-hmo/protocol.c",
     "libsigrok/src/hardware/hameg-hmo/protocol.h"], INDEPENDENT)
MAP["src/drivers/hameg_hmo/tests.rs"] = (
    ["libsigrok/src/hardware/hameg-hmo/api.c",
     "libsigrok/src/hardware/hameg-hmo/protocol.c"], INDEPENDENT)
