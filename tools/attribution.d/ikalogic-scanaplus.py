# The port of the driver (src/drivers/ikalogic_scanaplus).
MAP["src/drivers/ikalogic_scanaplus/mod.rs"] = (
    ["libsigrok/src/hardware/ikalogic-scanaplus/api.c"], INDEPENDENT)
MAP["src/drivers/ikalogic_scanaplus/protocol.rs"] = (
    ["libsigrok/src/hardware/ikalogic-scanaplus/protocol.c",
     "libsigrok/src/hardware/ikalogic-scanaplus/protocol.h"],
    INDEPENDENT)
