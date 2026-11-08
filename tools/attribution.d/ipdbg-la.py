# The port (src/drivers/ipdbg_la); see tools/attribution.py.
MAP["src/drivers/ipdbg_la/mod.rs"] = (
    ["libsigrok/src/hardware/ipdbg-la/api.c",
     "libsigrok/src/hardware/ipdbg-la/protocol.c"], INDEPENDENT)
MAP["src/drivers/ipdbg_la/protocol.rs"] = (
    ["libsigrok/src/hardware/ipdbg-la/api.c",
     "libsigrok/src/hardware/ipdbg-la/protocol.c",
     "libsigrok/src/hardware/ipdbg-la/protocol.h"], INDEPENDENT)
MAP["src/drivers/ipdbg_la/tests.rs"] = (
    ["libsigrok/src/hardware/ipdbg-la/protocol.c"], INDEPENDENT)
