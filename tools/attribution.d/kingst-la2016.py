# The port of the driver.
MAP["src/drivers/kingst_la2016/mod.rs"] = (
    ["libsigrok/src/hardware/kingst-la2016/api.c",
     "libsigrok/src/hardware/kingst-la2016/protocol.c (the acquisition)"],
    INDEPENDENT)
MAP["src/drivers/kingst_la2016/protocol.rs"] = (
    ["libsigrok/src/hardware/kingst-la2016/protocol.c",
     "libsigrok/src/hardware/kingst-la2016/protocol.h"],
    INDEPENDENT)
