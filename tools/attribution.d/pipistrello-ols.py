# tools/attribution.py.
# The port of the driver (src/drivers/pipistrello_ols).
MAP["src/drivers/pipistrello_ols/mod.rs"] = (
    ["libsigrok/src/hardware/pipistrello-ols/api.c"], INDEPENDENT)
MAP["src/drivers/pipistrello_ols/protocol.rs"] = (
    ["libsigrok/src/hardware/pipistrello-ols/protocol.c",
     "libsigrok/src/hardware/pipistrello-ols/protocol.h",
     "libsigrok/src/hardware/pipistrello-ols/api.c (the counts and the trigger stages)"],
    INDEPENDENT)
