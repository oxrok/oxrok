# The port (src/drivers/beaglelogic); see tools/attribution.py.
MAP["src/drivers/beaglelogic/mod.rs"] = (
    ["libsigrok/src/hardware/beaglelogic/api.c",
     "libsigrok/src/hardware/beaglelogic/beaglelogic_tcp.c",
     "libsigrok/src/hardware/beaglelogic/protocol.c"], INDEPENDENT)
MAP["src/drivers/beaglelogic/protocol.rs"] = (
    ["libsigrok/src/hardware/beaglelogic/beaglelogic_tcp.c",
     "libsigrok/src/hardware/beaglelogic/protocol.c",
     "libsigrok/src/hardware/beaglelogic/protocol.h"], INDEPENDENT)
MAP["src/drivers/beaglelogic/tests.rs"] = (
    ["libsigrok/src/hardware/beaglelogic/api.c",
     "libsigrok/src/hardware/beaglelogic/beaglelogic_tcp.c",
     "libsigrok/src/hardware/beaglelogic/protocol.c"], INDEPENDENT)
