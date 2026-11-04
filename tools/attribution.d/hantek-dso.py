# The hantek-dso device model (emu/src/models/hantek-dso.rs); see tools/attribution.py.
MAP["emu/src/models/hantek-dso.rs"] = (
    ["libsigrok/src/hardware/hantek-dso/api.c (the profiles, the scan, the frame transfer)",
     "libsigrok/src/hardware/hantek-dso/protocol.c (the commands, the capture state)",
     "libsigrok/src/hardware/hantek-dso/protocol.h"],
    INDEPENDENT)
# The port of the driver.
MAP["src/drivers/hantek_dso/mod.rs"] = (
    ["libsigrok/src/hardware/hantek-dso/api.c"],
    INDEPENDENT)
MAP["src/drivers/hantek_dso/protocol.rs"] = (
    ["libsigrok/src/hardware/hantek-dso/protocol.c",
     "libsigrok/src/hardware/hantek-dso/protocol.h",
     "libsigrok/src/hardware/hantek-dso/api.c (the tables, send_chunk(), receive_transfer())"],
    INDEPENDENT)
