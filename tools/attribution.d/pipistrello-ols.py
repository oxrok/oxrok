# The Pipistrello OLS device model (emu/src/models/pipistrello-ols.rs); see
# tools/attribution.py.
MAP["emu/src/models/pipistrello-ols.rs"] = (
    ["libsigrok/src/hardware/pipistrello-ols/api.c (the scan, the trigger stages, the counts)",
     "libsigrok/src/hardware/pipistrello-ols/protocol.c (the commands, the metadata, the sample stream)",
     "libsigrok/src/hardware/pipistrello-ols/protocol.h (the opcodes and flags)",
     "Pipistrello_ols_verilog 2423ef3, github.com/magnuskarlsson/Pipistrello_ols_verilog "
     "(verilog/stage.v, trigger.v, controller.v, rle_enc.v, core.v: stage matching, RLE masks, "
     "count widths)"], INDEPENDENT)
# The port of the driver (src/drivers/pipistrello_ols).
MAP["src/drivers/pipistrello_ols/mod.rs"] = (
    ["libsigrok/src/hardware/pipistrello-ols/api.c"], INDEPENDENT)
MAP["src/drivers/pipistrello_ols/protocol.rs"] = (
    ["libsigrok/src/hardware/pipistrello-ols/protocol.c",
     "libsigrok/src/hardware/pipistrello-ols/protocol.h",
     "libsigrok/src/hardware/pipistrello-ols/api.c (the counts and the trigger stages)"],
    INDEPENDENT)
