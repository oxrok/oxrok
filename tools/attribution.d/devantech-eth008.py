# The port (src/drivers/devantech_eth008, src/serial/tcpraw.rs); see tools/attribution.py.
MAP["src/drivers/devantech_eth008/mod.rs"] = (
    ["libsigrok/src/hardware/devantech-eth008/api.c",
     "libsigrok/src/hardware/devantech-eth008/protocol.c",
     "libsigrok/src/hardware/devantech-eth008/protocol.h"], INDEPENDENT)
MAP["src/drivers/devantech_eth008/protocol.rs"] = (
    ["libsigrok/src/hardware/devantech-eth008/api.c",
     "libsigrok/src/hardware/devantech-eth008/protocol.c",
     "libsigrok/src/hardware/devantech-eth008/protocol.h"], INDEPENDENT)
MAP["src/drivers/devantech_eth008/tests.rs"] = (
    ["libsigrok/src/hardware/devantech-eth008/api.c",
     "libsigrok/src/hardware/devantech-eth008/protocol.c"], INDEPENDENT)
MAP["src/serial/tcpraw.rs"] = (
    ["libsigrok/src/serial_tcpraw.c",
     "libsigrok/src/tcp.c"], INDEPENDENT)
