# The port of the driver (src/drivers/sysclk_lwla).
MAP["src/drivers/sysclk_lwla/mod.rs"] = (
    ["libsigrok/src/hardware/sysclk-lwla/api.c",
     "libsigrok/src/hardware/sysclk-lwla/protocol.c (the acquisition)"], INDEPENDENT)
MAP["src/drivers/sysclk_lwla/protocol.rs"] = (
    ["libsigrok/src/hardware/sysclk-lwla/lwla.c",
     "libsigrok/src/hardware/sysclk-lwla/lwla.h",
     "libsigrok/src/hardware/sysclk-lwla/lwla1016.c",
     "libsigrok/src/hardware/sysclk-lwla/lwla1034.c",
     "libsigrok/src/hardware/sysclk-lwla/protocol.c",
     "libsigrok/src/hardware/sysclk-lwla/protocol.h"], INDEPENDENT)
