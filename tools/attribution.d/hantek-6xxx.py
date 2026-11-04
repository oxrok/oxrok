# attribution.py credits files only: the sigrok-firmware-fx2lafw lines
# of the model's header (Ubixum, Jochen Hoenicke, Marek Wodzinski, from
# include/scope.inc, include/dscr_scope.inc and hw/*/fw.c) are written by
# hand after the generated ones and must be put back after a rerun.
# The port of the driver.
MAP["src/drivers/hantek_6xxx/mod.rs"] = (
    ["libsigrok/src/hardware/hantek-6xxx/api.c"],
    INDEPENDENT)
MAP["src/drivers/hantek_6xxx/protocol.rs"] = (
    ["libsigrok/src/hardware/hantek-6xxx/protocol.c",
     "libsigrok/src/hardware/hantek-6xxx/protocol.h"],
    INDEPENDENT)
