# The korad-kaxxxxp device model (emu/src/models/korad-kaxxxxp.rs); see tools/attribution.py.
MAP["emu/src/models/korad-kaxxxxp.rs"] = (
    ["libsigrok/src/hardware/korad-kaxxxxp/api.c (the supported models)",
     "libsigrok/src/hardware/korad-kaxxxxp/protocol.c (the commands and the status byte)"], INDEPENDENT)
# The port (src/drivers/korad_kaxxxxp).
MAP["src/drivers/korad_kaxxxxp/mod.rs"] = (
    ["libsigrok/src/hardware/korad-kaxxxxp/api.c"], INDEPENDENT)
MAP["src/drivers/korad_kaxxxxp/protocol.rs"] = (
    ["libsigrok/src/hardware/korad-kaxxxxp/api.c",
     "libsigrok/src/hardware/korad-kaxxxxp/protocol.c",
     "libsigrok/src/hardware/korad-kaxxxxp/protocol.h"], INDEPENDENT)
