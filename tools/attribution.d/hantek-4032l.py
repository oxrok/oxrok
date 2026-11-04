# The hantek-4032l device model (emu/src/models/hantek-4032l.rs); see tools/attribution.py.
MAP["emu/src/models/hantek-4032l.rs"] = (
    ["libsigrok/src/hardware/hantek-4032l/api.c (the rates, the trigger packing)",
     "libsigrok/src/hardware/hantek-4032l/protocol.c (the commands, the status and the stream)",
     "libsigrok/src/hardware/hantek-4032l/protocol.h"],
    INDEPENDENT)
# The port of the driver.
MAP["src/drivers/hantek_4032l/mod.rs"] = (
    ["libsigrok/src/hardware/hantek-4032l/api.c",
     "libsigrok/src/hardware/hantek-4032l/protocol.c (the capture)"],
    INDEPENDENT)
MAP["src/drivers/hantek_4032l/protocol.rs"] = (
    ["libsigrok/src/hardware/hantek-4032l/protocol.c",
     "libsigrok/src/hardware/hantek-4032l/protocol.h",
     "libsigrok/src/hardware/hantek-4032l/api.c (the rate and clock tables)"],
    INDEPENDENT)
