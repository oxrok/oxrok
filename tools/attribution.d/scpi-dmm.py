# The port (src/drivers/scpi_dmm); see tools/attribution.py.
MAP["src/drivers/scpi_dmm/mod.rs"] = (
    ["libsigrok/src/hardware/scpi-dmm/api.c",
     "libsigrok/src/hardware/scpi-dmm/protocol.c"], INDEPENDENT)
MAP["src/drivers/scpi_dmm/protocol.rs"] = (
    ["libsigrok/src/hardware/scpi-dmm/api.c",
     "libsigrok/src/hardware/scpi-dmm/protocol.c",
     "libsigrok/src/hardware/scpi-dmm/protocol.h"], INDEPENDENT)
MAP["src/drivers/scpi_dmm/tests.rs"] = (
    ["libsigrok/src/hardware/scpi-dmm/api.c",
     "libsigrok/src/hardware/scpi-dmm/protocol.c"], INDEPENDENT)
