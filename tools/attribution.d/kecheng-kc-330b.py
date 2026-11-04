# The kecheng-kc-330b device model (emu/src/models/kecheng-kc-330b.rs); see tools/attribution.py.
MAP["emu/src/models/kecheng-kc-330b.rs"] = (
    ["libsigrok/src/hardware/kecheng-kc-330b/api.c (the identify, what its comments say of the device)",
     "libsigrok/src/hardware/kecheng-kc-330b/protocol.c (the commands and answers)",
     "libsigrok/src/hardware/kecheng-kc-330b/protocol.h"],
    INDEPENDENT)

# The port of the driver (src/drivers/kecheng_kc_330b).
MAP["src/drivers/kecheng_kc_330b/mod.rs"] = (
    ["libsigrok/src/hardware/kecheng-kc-330b/api.c",
     "libsigrok/src/hardware/kecheng-kc-330b/protocol.c (the acquisition)"], INDEPENDENT)
MAP["src/drivers/kecheng_kc_330b/protocol.rs"] = (
    ["libsigrok/src/hardware/kecheng-kc-330b/protocol.c",
     "libsigrok/src/hardware/kecheng-kc-330b/protocol.h",
     "libsigrok/src/hardware/kecheng-kc-330b/api.c (the intervals, the identify)"], INDEPENDENT)
