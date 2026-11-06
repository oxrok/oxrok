# The port (src/drivers/link_mso19); see tools/attribution.py.
MAP["src/drivers/link_mso19/mod.rs"] = (
    ["libsigrok/src/hardware/link-mso19/api.c",
     "libsigrok/src/hardware/link-mso19/protocol.c"], INDEPENDENT)
MAP["src/drivers/link_mso19/protocol.rs"] = (
    ["libsigrok/src/hardware/link-mso19/api.c",
     "libsigrok/src/hardware/link-mso19/protocol.c",
     "libsigrok/src/hardware/link-mso19/protocol.h"], INDEPENDENT)
MAP["src/drivers/link_mso19/tests.rs"] = (
    ["libsigrok/src/hardware/link-mso19/protocol.c"], INDEPENDENT)
