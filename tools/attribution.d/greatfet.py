# The greatfet device model and port; see tools/attribution.py.
MAP["src/drivers/greatfet/mod.rs"] = (
    ["libsigrok/src/hardware/greatfet/api.c",
     "libsigrok/src/hardware/greatfet/protocol.c (the transfers, the sample data)"],
    INDEPENDENT)
MAP["src/drivers/greatfet/protocol.rs"] = (
    ["libsigrok/src/hardware/greatfet/protocol.c",
     "libsigrok/src/hardware/greatfet/protocol.h"],
    INDEPENDENT)
MAP["emu/src/models/greatfet.rs"] = (
    ["libsigrok/src/hardware/greatfet/api.c (the scan, the channels, the rates)",
     "libsigrok/src/hardware/greatfet/protocol.c (the commands and the transfers)",
     "libsigrok/src/hardware/greatfet/protocol.h"],
    INDEPENDENT)
