# The saleae-logic16 device model (emu/src/models/saleae-logic16.rs); see tools/attribution.py.
MAP["emu/src/models/saleae-logic16.rs"] = (
    ["libsigrok/src/hardware/saleae-logic16/api.c (the ids, strings, rates, transfers)",
     "libsigrok/src/hardware/saleae-logic16/protocol.c (the EP1 commands, their scrambling, "
     "the FPGA register maps, the primer, the sample format)",
     "libsigrok/src/hardware/saleae-logic16/protocol.h"],
    INDEPENDENT)
# The port of the driver.
MAP["src/drivers/saleae_logic16/mod.rs"] = (
    ["libsigrok/src/hardware/saleae-logic16/api.c",
     "libsigrok/src/hardware/saleae-logic16/protocol.c (the acquisition)"],
    INDEPENDENT)
MAP["src/drivers/saleae_logic16/protocol.rs"] = (
    ["libsigrok/src/hardware/saleae-logic16/protocol.c",
     "libsigrok/src/hardware/saleae-logic16/protocol.h"],
    INDEPENDENT)
