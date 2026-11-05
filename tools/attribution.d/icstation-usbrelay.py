# The port (src/drivers/icstation_usbrelay); see tools/attribution.py.
MAP["src/drivers/icstation_usbrelay/mod.rs"] = (
    ["libsigrok/src/hardware/icstation-usbrelay/api.c",
     "libsigrok/src/hardware/icstation-usbrelay/protocol.c"], INDEPENDENT)
MAP["src/drivers/icstation_usbrelay/protocol.rs"] = (
    ["libsigrok/src/hardware/icstation-usbrelay/api.c",
     "libsigrok/src/hardware/icstation-usbrelay/protocol.c",
     "libsigrok/src/hardware/icstation-usbrelay/protocol.h"], INDEPENDENT)
MAP["src/drivers/icstation_usbrelay/tests.rs"] = (
    ["libsigrok/src/hardware/icstation-usbrelay/protocol.c"], INDEPENDENT)
