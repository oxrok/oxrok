MAP["src/drivers/lascar_el_usb/mod.rs"] = (
    ["libsigrok/src/hardware/lascar-el-usb/api.c",
     "libsigrok/src/hardware/lascar-el-usb/protocol.c (the receive callback)",
     "eltuxusb (Romain Aviolat), GPLv3, version 0.3"],
    INDEPENDENT)
MAP["src/drivers/lascar_el_usb/protocol.rs"] = (
    ["libsigrok/src/hardware/lascar-el-usb/protocol.c",
     "libsigrok/src/hardware/lascar-el-usb/protocol.h",
     "libsigrok/src/hardware/lascar-el-usb/api.c (the log request, lascar_proc_config())"],
    INDEPENDENT)
