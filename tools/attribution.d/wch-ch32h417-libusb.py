# The port of the driver (src/drivers/wch_ch32h417_libusb). Its C is not
# sigrok's: the WeAct Studio tree's, which oxrok/libsigrok carries, so
# SIGROK_SRC/libsigrok must be a checkout of that fork's oxrok branch.
MAP["src/drivers/wch_ch32h417_libusb/mod.rs"] = (
    ["libsigrok/src/hardware/wch-ch32h417-libusb/api.c",
     "libsigrok/src/hardware/wch-ch32h417-libusb/protocol.c (the acquisition)",
     "libsigrok/src/hardware/wch-ch32h417-libusb/libusb_wrapper.c (the scan and the open)"],
    INDEPENDENT)
MAP["src/drivers/wch_ch32h417_libusb/protocol.rs"] = (
    ["libsigrok/src/hardware/wch-ch32h417-libusb/protocol.c",
     "libsigrok/src/hardware/wch-ch32h417-libusb/protocol.h",
     "libsigrok/src/hardware/wch-ch32h417-libusb/api.c (the boards, the threshold)",
     "libsigrok/src/hardware/wch-ch32h417-libusb/libusb_wrapper.h (the sample transfers)"],
    INDEPENDENT)
