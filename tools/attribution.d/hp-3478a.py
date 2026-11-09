# The port (src/drivers/hp_3478a) and the GPIB transport; see tools/attribution.py.
MAP["src/drivers/hp_3478a/mod.rs"] = (
    ["libsigrok/src/hardware/hp-3478a/api.c",
     "libsigrok/src/hardware/hp-3478a/protocol.c"], INDEPENDENT)
MAP["src/drivers/hp_3478a/protocol.rs"] = (
    ["libsigrok/src/hardware/hp-3478a/protocol.c",
     "libsigrok/src/hardware/hp-3478a/protocol.h"], INDEPENDENT)
MAP["src/drivers/hp_3478a/tests.rs"] = (
    ["libsigrok/src/hardware/hp-3478a/api.c",
     "libsigrok/src/hardware/hp-3478a/protocol.c"], INDEPENDENT)
MAP["src/gpib/mod.rs"] = (["libsigrok/src/scpi/scpi_libgpib.c"], INDEPENDENT)
MAP["src/gpib/sys.rs"] = (["libsigrok/src/scpi/scpi_libgpib.c"], INDEPENDENT)
MAP["src/scpi/gpib.rs"] = (["libsigrok/src/scpi/scpi_libgpib.c"], INDEPENDENT)
