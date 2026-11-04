# The port of the driver (src/drivers/sysclk_sla5032).
MAP["src/drivers/sysclk_sla5032/mod.rs"] = (
    ["libsigrok/src/hardware/sysclk-sla5032/api.c",
     "libsigrok/src/hardware/sysclk-sla5032/protocol.c (the acquisition)"], INDEPENDENT)
MAP["src/drivers/sysclk_sla5032/protocol.rs"] = (
    ["libsigrok/src/hardware/sysclk-sla5032/protocol.c",
     "libsigrok/src/hardware/sysclk-sla5032/protocol.h"], INDEPENDENT)
