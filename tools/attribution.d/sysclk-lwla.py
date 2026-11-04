# The sysclk-lwla device model (emu/src/models/sysclk-lwla/); see tools/attribution.py.
MAP["emu/src/models/sysclk-lwla/mod.rs"] = (
    ["libsigrok/src/hardware/sysclk-lwla/api.c (the ids, the endpoints, the open's drain)",
     "libsigrok/src/hardware/sysclk-lwla/lwla.c (the bitstream transfer, the command words)",
     "libsigrok/src/hardware/sysclk-lwla/lwla.h",
     "libsigrok/src/hardware/sysclk-lwla/lwla1034.c (the registers, long registers and the "
     "36-bit memory format)",
     "libsigrok/src/hardware/sysclk-lwla/lwla1016.c (the registers and the 32-bit memory "
     "formats)",
     "libsigrok/src/hardware/sysclk-lwla/protocol.c (the capture sequence and the status poll)",
     "libsigrok/src/hardware/sysclk-lwla/protocol.h"],
    INDEPENDENT)
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
