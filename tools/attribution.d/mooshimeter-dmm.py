# The port (src/drivers/mooshimeter_dmm) and its BLE transport (src/bt); see
# tools/attribution.py.
MAP["src/bt/mod.rs"] = (["libsigrok/src/bt/bt_bluez.c"], INDEPENDENT)
MAP["src/drivers/mooshimeter_dmm/mod.rs"] = (
    ["libsigrok/src/hardware/mooshimeter-dmm/api.c",
     "libsigrok/src/hardware/mooshimeter-dmm/protocol.c"], INDEPENDENT)
MAP["src/drivers/mooshimeter_dmm/protocol.rs"] = (
    ["libsigrok/src/hardware/mooshimeter-dmm/protocol.c",
     "libsigrok/src/hardware/mooshimeter-dmm/protocol.h"], INDEPENDENT)
MAP["src/drivers/mooshimeter_dmm/tests.rs"] = (
    ["libsigrok/src/hardware/mooshimeter-dmm/api.c",
     "libsigrok/src/hardware/mooshimeter-dmm/protocol.c"], INDEPENDENT)
