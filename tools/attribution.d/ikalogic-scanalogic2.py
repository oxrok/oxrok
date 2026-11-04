# The ikalogic-scanalogic2 device model (emu/src/models/ikalogic-scanalogic2.rs); see tools/attribution.py.
MAP["emu/src/models/ikalogic-scanalogic2.rs"] = (
    ["libsigrok/src/hardware/ikalogic-scanalogic2/api.c (the open sequence, the capture command)",
     "libsigrok/src/hardware/ikalogic-scanalogic2/protocol.c (the status reads, the sample packets)",
     "libsigrok/src/hardware/ikalogic-scanalogic2/protocol.h"],
    INDEPENDENT)
# The port of the driver.
MAP["src/drivers/ikalogic_scanalogic2/mod.rs"] = (
    ["libsigrok/src/hardware/ikalogic-scanalogic2/api.c"],
    INDEPENDENT)
MAP["src/drivers/ikalogic_scanalogic2/protocol.rs"] = (
    ["libsigrok/src/hardware/ikalogic-scanalogic2/api.c (the open sequence, the capture command)",
     "libsigrok/src/hardware/ikalogic-scanalogic2/protocol.c",
     "libsigrok/src/hardware/ikalogic-scanalogic2/protocol.h"],
    INDEPENDENT)
