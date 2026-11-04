# The dreamsourcelab-dslogic device model
# (emu/src/models/dreamsourcelab-dslogic.rs); see tools/attribution.py.
MAP["emu/src/models/dreamsourcelab-dslogic.rs"] = (
    ["libsigrok/src/hardware/dreamsourcelab-dslogic/api.c (the profiles, the scan, the open)",
     "libsigrok/src/hardware/dreamsourcelab-dslogic/protocol.c (the commands, the FPGA setting packet, the trigger packet and the sample transfers)",
     "libsigrok/src/hardware/dreamsourcelab-dslogic/protocol.h"],
    INDEPENDENT)
# The port of the driver.
MAP["src/drivers/dreamsourcelab_dslogic/mod.rs"] = (
    ["libsigrok/src/hardware/dreamsourcelab-dslogic/api.c",
     "libsigrok/src/hardware/dreamsourcelab-dslogic/protocol.c (the acquisition)"],
    INDEPENDENT)
MAP["src/drivers/dreamsourcelab_dslogic/protocol.rs"] = (
    ["libsigrok/src/hardware/dreamsourcelab-dslogic/protocol.c",
     "libsigrok/src/hardware/dreamsourcelab-dslogic/protocol.h"],
    INDEPENDENT)
