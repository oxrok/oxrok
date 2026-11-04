# tools/attribution.py.
# The port of the driver (src/drivers/chronovu_la).
MAP["src/drivers/chronovu_la/mod.rs"] = (
    ["libsigrok/src/hardware/chronovu-la/api.c"], INDEPENDENT)
MAP["src/drivers/chronovu_la/protocol.rs"] = (
    ["libsigrok/src/hardware/chronovu-la/protocol.c",
     "libsigrok/src/hardware/chronovu-la/protocol.h"], INDEPENDENT)
