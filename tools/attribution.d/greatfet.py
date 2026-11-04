# Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
# The greatfet port; see tools/attribution.py.
MAP["src/drivers/greatfet/mod.rs"] = (
    ["libsigrok/src/hardware/greatfet/api.c",
     "libsigrok/src/hardware/greatfet/protocol.c (the transfers, the sample data)"],
    INDEPENDENT)
MAP["src/drivers/greatfet/protocol.rs"] = (
    ["libsigrok/src/hardware/greatfet/protocol.c",
     "libsigrok/src/hardware/greatfet/protocol.h"],
    INDEPENDENT)
