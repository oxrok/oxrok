# The port (src/drivers/atten_pps3xxx); see tools/attribution.py.
MAP["src/drivers/atten_pps3xxx/mod.rs"] = (
    ["libsigrok/src/hardware/atten-pps3xxx/api.c",
     "libsigrok/src/hardware/atten-pps3xxx/protocol.c"], INDEPENDENT)
MAP["src/drivers/atten_pps3xxx/protocol.rs"] = (
    ["libsigrok/src/hardware/atten-pps3xxx/protocol.c",
     "libsigrok/src/hardware/atten-pps3xxx/protocol.h",
     "libsigrok/src/hardware/atten-pps3xxx/api.c (the models table)"], INDEPENDENT)
