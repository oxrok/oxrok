# The asix-sigma device model (emu/src/models/asix-sigma/); see tools/attribution.py.
MAP["emu/src/models/asix-sigma/mod.rs"] = (
    ["libsigrok/src/hardware/asix-sigma/api.c (the ids, the scan by serial number)",
     "libsigrok/src/hardware/asix-sigma/protocol.c (the bit-bang pins, the netlist scrambling, "
     "the FPGA configuration sequence)",
     "libsigrok/src/hardware/asix-sigma/protocol.h"], INDEPENDENT)
MAP["emu/src/models/asix-sigma/fpga.rs"] = (
    ["libsigrok/src/hardware/asix-sigma/api.c (the clock select, trigger select 2 and post "
     "trigger values of dev_acquisition_start)",
     "libsigrok/src/hardware/asix-sigma/protocol.c (the FPGA commands, registers, trigger table "
     "and sample memory layout)",
     "libsigrok/src/hardware/asix-sigma/protocol.h"], INDEPENDENT)
