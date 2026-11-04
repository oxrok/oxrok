# The sysclk-sla5032 device model (emu/src/models/sysclk-sla5032.rs); see tools/attribution.py.
MAP["emu/src/models/sysclk-sla5032.rs"] = (
    ["libsigrok/src/hardware/sysclk-sla5032/api.c (the ids, the open)",
     "libsigrok/src/hardware/sysclk-sla5032/protocol.c (the commands, the registers, the "
     "bitstream's framing, the capture sequence and the RLE records)",
     "libsigrok/src/hardware/sysclk-sla5032/protocol.h"],
    INDEPENDENT)
# The port of the driver (src/drivers/sysclk_sla5032).
MAP["src/drivers/sysclk_sla5032/mod.rs"] = (
    ["libsigrok/src/hardware/sysclk-sla5032/api.c",
     "libsigrok/src/hardware/sysclk-sla5032/protocol.c (the acquisition)"], INDEPENDENT)
MAP["src/drivers/sysclk_sla5032/protocol.rs"] = (
    ["libsigrok/src/hardware/sysclk-sla5032/protocol.c",
     "libsigrok/src/hardware/sysclk-sla5032/protocol.h"], INDEPENDENT)
