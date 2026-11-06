# The port (src/drivers/kern_scale); see tools/attribution.py.
MAP["src/drivers/kern_scale/mod.rs"] = (
    ["libsigrok/src/hardware/kern-scale/api.c",
     "libsigrok/src/hardware/kern-scale/protocol.c",
     "libsigrok/src/hardware/kern-scale/protocol.h"], INDEPENDENT)
MAP["src/drivers/kern_scale/protocol.rs"] = (
    ["libsigrok/src/scale/kern.c"], INDEPENDENT)
MAP["src/drivers/kern_scale/tests.rs"] = (
    ["libsigrok/src/hardware/kern-scale/api.c",
     "libsigrok/src/hardware/kern-scale/protocol.c"], INDEPENDENT)
