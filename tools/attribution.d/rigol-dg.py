# The port (src/drivers/rigol_dg); see tools/attribution.py.
MAP["src/drivers/rigol_dg/mod.rs"] = (
    ["libsigrok/src/hardware/rigol-dg/api.c"], INDEPENDENT)
MAP["src/drivers/rigol_dg/protocol.rs"] = (
    ["libsigrok/src/hardware/rigol-dg/api.c",
     "libsigrok/src/hardware/rigol-dg/protocol.c",
     "libsigrok/src/hardware/rigol-dg/protocol.h"], INDEPENDENT)
