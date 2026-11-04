# The port of the driver.
MAP["src/drivers/saleae_logic_pro/mod.rs"] = (
    ["libsigrok/src/hardware/saleae-logic-pro/api.c",
     "libsigrok/src/hardware/saleae-logic-pro/protocol.c (the receive callback)"],
    INDEPENDENT)
MAP["src/drivers/saleae_logic_pro/protocol.rs"] = (
    ["libsigrok/src/hardware/saleae-logic-pro/api.c (the firmware upload)",
     "libsigrok/src/hardware/saleae-logic-pro/protocol.c",
     "libsigrok/src/hardware/saleae-logic-pro/protocol.h"],
    INDEPENDENT)
