# The port of the driver.
MAP["src/drivers/hantek_dso/mod.rs"] = (
    ["libsigrok/src/hardware/hantek-dso/api.c"],
    INDEPENDENT)
MAP["src/drivers/hantek_dso/protocol.rs"] = (
    ["libsigrok/src/hardware/hantek-dso/protocol.c",
     "libsigrok/src/hardware/hantek-dso/protocol.h",
     "libsigrok/src/hardware/hantek-dso/api.c (the tables, send_chunk(), receive_transfer())"],
    INDEPENDENT)
