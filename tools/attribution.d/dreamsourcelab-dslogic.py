# The port of the driver.
MAP["src/drivers/dreamsourcelab_dslogic/mod.rs"] = (
    ["libsigrok/src/hardware/dreamsourcelab-dslogic/api.c",
     "libsigrok/src/hardware/dreamsourcelab-dslogic/protocol.c (the acquisition)"],
    INDEPENDENT)
MAP["src/drivers/dreamsourcelab_dslogic/protocol.rs"] = (
    ["libsigrok/src/hardware/dreamsourcelab-dslogic/protocol.c",
     "libsigrok/src/hardware/dreamsourcelab-dslogic/protocol.h"],
    INDEPENDENT)
