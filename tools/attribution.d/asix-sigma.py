# The port of the driver (src/drivers/asix_sigma).
MAP["src/drivers/asix_sigma/mod.rs"] = (
    ["libsigrok/src/hardware/asix-sigma/api.c"], INDEPENDENT)
MAP["src/drivers/asix_sigma/protocol.rs"] = (
    ["libsigrok/src/hardware/asix-sigma/protocol.c",
     "libsigrok/src/hardware/asix-sigma/protocol.h",
     "libsigrok/src/hardware/asix-sigma/api.c (dev_acquisition_start's register values)"],
    INDEPENDENT)
