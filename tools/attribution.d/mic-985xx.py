# The port (src/drivers/mic_985xx); see tools/attribution.py.
MAP["src/drivers/mic_985xx/mod.rs"] = (
    ["libsigrok/src/hardware/mic-985xx/api.c",
     "libsigrok/src/hardware/mic-985xx/protocol.c"], INDEPENDENT)
MAP["src/drivers/mic_985xx/protocol.rs"] = (
    ["libsigrok/src/hardware/mic-985xx/api.c",
     "libsigrok/src/hardware/mic-985xx/protocol.c",
     "libsigrok/src/hardware/mic-985xx/protocol.h"], INDEPENDENT)
MAP["src/drivers/mic_985xx/tests.rs"] = (
    ["libsigrok/src/hardware/mic-985xx/protocol.c"], INDEPENDENT)
