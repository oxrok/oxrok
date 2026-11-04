# The kingst-la2016 device model (emu/src/models/kingst-la2016/); see tools/attribution.py.
MAP["emu/src/models/kingst-la2016/mod.rs"] = (
    ["libsigrok/src/hardware/kingst-la2016/api.c (the models, rates, thresholds, the scan)",
     "libsigrok/src/hardware/kingst-la2016/protocol.c (the requests, registers and sample layout)",
     "libsigrok/src/hardware/kingst-la2016/protocol.h"], INDEPENDENT)
MAP["emu/src/models/kingst-la2016/capture.rs"] = (
    ["libsigrok/src/hardware/kingst-la2016/protocol.c (send_chunk(), stream_data(): the sample layouts)"],
    INDEPENDENT)
# The port of the driver.
MAP["src/drivers/kingst_la2016/mod.rs"] = (
    ["libsigrok/src/hardware/kingst-la2016/api.c",
     "libsigrok/src/hardware/kingst-la2016/protocol.c (the acquisition)"],
    INDEPENDENT)
MAP["src/drivers/kingst_la2016/protocol.rs"] = (
    ["libsigrok/src/hardware/kingst-la2016/protocol.c",
     "libsigrok/src/hardware/kingst-la2016/protocol.h"],
    INDEPENDENT)
