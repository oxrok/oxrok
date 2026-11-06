# The port (src/drivers/scpi_pps); see tools/attribution.py.
MAP["src/drivers/scpi_pps/mod.rs"] = (
    ["libsigrok/src/hardware/scpi-pps/api.c",
     "libsigrok/src/hardware/scpi-pps/protocol.c"], INDEPENDENT)
MAP["src/drivers/scpi_pps/protocol.rs"] = (
    ["libsigrok/src/hardware/scpi-pps/api.c",
     "libsigrok/src/hardware/scpi-pps/protocol.c",
     "libsigrok/src/hardware/scpi-pps/protocol.h",
     "libsigrok/src/hardware/scpi-pps/profiles.c"], INDEPENDENT)
MAP["src/drivers/scpi_pps/profiles.rs"] = (
    ["libsigrok/src/hardware/scpi-pps/profiles.c"], INDEPENDENT)
MAP["src/drivers/scpi_pps/tests.rs"] = (
    ["libsigrok/src/hardware/scpi-pps/api.c",
     "libsigrok/src/hardware/scpi-pps/protocol.c"], INDEPENDENT)
