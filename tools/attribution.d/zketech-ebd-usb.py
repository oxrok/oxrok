# The port (src/drivers/zketech_ebd_usb); see tools/attribution.py.
MAP["src/drivers/zketech_ebd_usb/mod.rs"] = (
    ["libsigrok/src/hardware/zketech-ebd-usb/api.c",
     "libsigrok/src/hardware/zketech-ebd-usb/protocol.c"], INDEPENDENT)
MAP["src/drivers/zketech_ebd_usb/protocol.rs"] = (
    ["libsigrok/src/hardware/zketech-ebd-usb/protocol.c",
     "libsigrok/src/hardware/zketech-ebd-usb/protocol.h"], INDEPENDENT)
MAP["src/drivers/zketech_ebd_usb/tests.rs"] = (
    ["libsigrok/src/hardware/zketech-ebd-usb/protocol.c"], INDEPENDENT)
