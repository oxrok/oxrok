# The port (src/drivers/lecroy_xstream); see tools/attribution.py.
MAP["src/drivers/lecroy_xstream/mod.rs"] = (
    ["libsigrok/src/hardware/lecroy-xstream/api.c",
     "libsigrok/src/hardware/lecroy-xstream/protocol.c",
     "libsigrok/src/hardware/lecroy-xstream/protocol.h"], INDEPENDENT)
MAP["src/drivers/lecroy_xstream/protocol.rs"] = (
    ["libsigrok/src/hardware/lecroy-xstream/protocol.c",
     "libsigrok/src/hardware/lecroy-xstream/protocol.h"], INDEPENDENT)
MAP["src/drivers/lecroy_xstream/tests.rs"] = (
    ["libsigrok/src/hardware/lecroy-xstream/api.c",
     "libsigrok/src/hardware/lecroy-xstream/protocol.c"], INDEPENDENT)
