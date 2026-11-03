# The hantek-6xxx device model (emu/src/models/hantek-6xxx.rs); see tools/attribution.py.
# attribution.py credits files only: the sigrok-firmware-fx2lafw lines
# of the model's header (Ubixum, Jochen Hoenicke, Marek Wodzinski, from
# include/scope.inc, include/dscr_scope.inc and hw/*/fw.c) are written by
# hand after the generated ones and must be put back after a rerun.
MAP["emu/src/models/hantek-6xxx.rs"] = (
    ["libsigrok/src/hardware/hantek-6xxx/api.c (the profiles, the scan, the transfers)",
     "libsigrok/src/hardware/hantek-6xxx/protocol.c (the requests)",
     "libsigrok/src/hardware/hantek-6xxx/protocol.h",
     "sigrok-firmware-fx2lafw (include/scope.inc, include/dscr_scope.inc, hw/*/fw.c, hw/*/dscr.a51, Makefile.am)"],
    INDEPENDENT)
