# The port of the driver.
MAP["src/drivers/saleae_logic16/mod.rs"] = (
    ["libsigrok/src/hardware/saleae-logic16/api.c",
     "libsigrok/src/hardware/saleae-logic16/protocol.c (the acquisition)"],
    INDEPENDENT)
MAP["src/drivers/saleae_logic16/protocol.rs"] = (
    ["libsigrok/src/hardware/saleae-logic16/protocol.c",
     "libsigrok/src/hardware/saleae-logic16/protocol.h"],
    INDEPENDENT)
