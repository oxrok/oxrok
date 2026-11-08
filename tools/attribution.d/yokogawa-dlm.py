# The port (src/drivers/yokogawa_dlm); see tools/attribution.py.
MAP["src/drivers/yokogawa_dlm/mod.rs"] = (
    ["libsigrok/src/hardware/yokogawa-dlm/api.c",
     "libsigrok/src/hardware/yokogawa-dlm/protocol.c",
     "libsigrok/src/hardware/yokogawa-dlm/protocol_wrappers.c"], INDEPENDENT)
MAP["src/drivers/yokogawa_dlm/protocol.rs"] = (
    ["libsigrok/src/hardware/yokogawa-dlm/protocol.c",
     "libsigrok/src/hardware/yokogawa-dlm/protocol.h",
     "libsigrok/src/hardware/yokogawa-dlm/protocol_wrappers.c"], INDEPENDENT)
MAP["src/drivers/yokogawa_dlm/tests.rs"] = (
    ["libsigrok/src/hardware/yokogawa-dlm/api.c",
     "libsigrok/src/hardware/yokogawa-dlm/protocol.c"], INDEPENDENT)
