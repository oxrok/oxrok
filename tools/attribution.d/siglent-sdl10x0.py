# The port (src/drivers/siglent_sdl10x0); see tools/attribution.py.
MAP["src/drivers/siglent_sdl10x0/mod.rs"] = (
    ["libsigrok/src/hardware/siglent-sdl10x0/api.c",
     "libsigrok/src/hardware/siglent-sdl10x0/protocol.c"], INDEPENDENT)
MAP["src/drivers/siglent_sdl10x0/protocol.rs"] = (
    ["libsigrok/src/hardware/siglent-sdl10x0/api.c",
     "libsigrok/src/hardware/siglent-sdl10x0/protocol.c",
     "libsigrok/src/hardware/siglent-sdl10x0/protocol.h"], INDEPENDENT)
MAP["src/drivers/siglent_sdl10x0/tests.rs"] = (
    ["libsigrok/src/hardware/siglent-sdl10x0/protocol.c"], INDEPENDENT)
