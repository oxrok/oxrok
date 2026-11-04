# The testo device model (emu/src/models/testo.rs) and the port
# (src/drivers/testo); see tools/attribution.py.
MAP["emu/src/models/testo.rs"] = (
    ["libsigrok/src/hardware/testo/api.c (the scan, the transfers)",
     "libsigrok/src/hardware/testo/protocol.c (the request and the answer's layout)",
     "libsigrok/src/hardware/testo/protocol.h"], INDEPENDENT)
MAP["src/drivers/testo/mod.rs"] = (
    ["libsigrok/src/hardware/testo/api.c"], INDEPENDENT)
MAP["src/drivers/testo/protocol.rs"] = (
    ["libsigrok/src/hardware/testo/protocol.c", "libsigrok/src/hardware/testo/protocol.h",
     "libsigrok/src/hardware/testo/api.c (receive_data, receive_transfer)"], INDEPENDENT)
