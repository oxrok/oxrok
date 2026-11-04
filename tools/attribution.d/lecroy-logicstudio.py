# The lecroy-logicstudio device model (emu/src/models/lecroy-logicstudio.rs); see tools/attribution.py.
MAP["emu/src/models/lecroy-logicstudio.rs"] = (
    ["libsigrok/src/hardware/lecroy-logicstudio/api.c (the ids, the firmware, the renumeration wait)",
     "libsigrok/src/hardware/lecroy-logicstudio/protocol.c (the requests, registers, bitstream "
     "upload, trigger layout, status packets, sample memory)",
     "libsigrok/src/hardware/lecroy-logicstudio/protocol.h"],
    INDEPENDENT)
MAP["src/drivers/lecroy_logicstudio/mod.rs"] = (
    ["libsigrok/src/hardware/lecroy-logicstudio/api.c",
     "libsigrok/src/hardware/lecroy-logicstudio/protocol.c (the acquisition)"],
    INDEPENDENT)
MAP["src/drivers/lecroy_logicstudio/protocol.rs"] = (
    ["libsigrok/src/hardware/lecroy-logicstudio/protocol.c",
     "libsigrok/src/hardware/lecroy-logicstudio/protocol.h"],
    INDEPENDENT)
