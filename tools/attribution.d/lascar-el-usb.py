# The lascar-el-usb device model (emu/src/models/lascar-el-usb.rs); see tools/attribution.py.
MAP["emu/src/models/lascar-el-usb.rs"] = (
    ["libsigrok/src/hardware/lascar-el-usb/protocol.c (the commands, the configuration block, the sample bytes)",
     "libsigrok/src/hardware/lascar-el-usb/protocol.h",
     "libsigrok/src/hardware/lascar-el-usb/api.c (the log download)",
     "eltuxusb (Romain Aviolat), GPL-3.0-or-later, version 0.3"],
    INDEPENDENT)
MAP["src/drivers/lascar_el_usb/mod.rs"] = (
    ["libsigrok/src/hardware/lascar-el-usb/api.c",
     "libsigrok/src/hardware/lascar-el-usb/protocol.c (the receive callback)",
     "eltuxusb (Romain Aviolat), GPL-3.0-or-later, version 0.3"],
    INDEPENDENT)
MAP["src/drivers/lascar_el_usb/protocol.rs"] = (
    ["libsigrok/src/hardware/lascar-el-usb/protocol.c",
     "libsigrok/src/hardware/lascar-el-usb/protocol.h",
     "libsigrok/src/hardware/lascar-el-usb/api.c (the log request, lascar_proc_config())"],
    INDEPENDENT)
