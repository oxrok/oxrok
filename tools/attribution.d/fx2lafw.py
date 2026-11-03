# The fx2lafw device model and the EZ-USB shell (emu/src/models/fx2lafw.rs,
# emu/src/shells/ezusb.rs); see tools/attribution.py.
MAP["emu/src/models/fx2lafw.rs"] = (
    ["libsigrok/src/hardware/fx2lafw/api.c (the profiles, the scan)",
     "libsigrok/src/hardware/fx2lafw/protocol.c (the commands and the transfers)",
     "libsigrok/src/hardware/fx2lafw/protocol.h",
     "sigrok-firmware-fx2lafw (fx2lafw.c, gpif-acquisition.c, include/dscr.inc, hw/*/dscr.a51)"],
    INDEPENDENT)
MAP["emu/src/shells/ezusb.rs"] = (
    ["libsigrok/src/ezusb.c (the upload sequence the checks expect)"], INDEPENDENT)
