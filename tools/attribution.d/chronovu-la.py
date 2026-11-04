# The chronovu-la device model (emu/src/models/chronovu-la.rs); see
# tools/attribution.py.
MAP["emu/src/models/chronovu-la.rs"] = (
    ["libsigrok/src/hardware/chronovu-la/api.c (the ids, the command and the sequence)",
     "libsigrok/src/hardware/chronovu-la/protocol.c (the command, the sequencer reset and the "
     "readout order)",
     "libsigrok/src/hardware/chronovu-la/protocol.h (the memory and the trigger masks)"],
    INDEPENDENT)
# The port of the driver (src/drivers/chronovu_la).
MAP["src/drivers/chronovu_la/mod.rs"] = (
    ["libsigrok/src/hardware/chronovu-la/api.c"], INDEPENDENT)
MAP["src/drivers/chronovu_la/protocol.rs"] = (
    ["libsigrok/src/hardware/chronovu-la/protocol.c",
     "libsigrok/src/hardware/chronovu-la/protocol.h"], INDEPENDENT)
