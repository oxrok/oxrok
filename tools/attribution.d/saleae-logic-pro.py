# The saleae-logic-pro device model (emu/src/models/saleae-logic-pro.rs); see tools/attribution.py.
MAP["emu/src/models/saleae-logic-pro.rs"] = (
    ["libsigrok/src/hardware/saleae-logic-pro/api.c (the ids, strings, firmware container, transfers)",
     "libsigrok/src/hardware/saleae-logic-pro/protocol.c (the scrambled commands, the LFSR, "
     "the crypto chip packets, the registers, the sample format)",
     "libsigrok/src/hardware/saleae-logic-pro/protocol.h"],
    INDEPENDENT)
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
