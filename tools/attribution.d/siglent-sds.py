# The port (src/drivers/siglent_sds); see tools/attribution.py.
MAP["src/drivers/siglent_sds/mod.rs"] = (
    ["libsigrok/src/hardware/siglent-sds/api.c",
     "libsigrok/src/hardware/siglent-sds/protocol.c",
     "libsigrok/src/hardware/siglent-sds/protocol.h"], INDEPENDENT)
MAP["src/drivers/siglent_sds/models.rs"] = (
    ["libsigrok/src/hardware/siglent-sds/api.c",
     "libsigrok/src/hardware/siglent-sds/protocol.h"], INDEPENDENT)
MAP["src/drivers/siglent_sds/protocol.rs"] = (
    ["libsigrok/src/hardware/siglent-sds/protocol.c",
     "libsigrok/src/hardware/siglent-sds/protocol.h"], INDEPENDENT)
MAP["src/drivers/siglent_sds/tests.rs"] = (
    ["libsigrok/src/hardware/siglent-sds/api.c",
     "libsigrok/src/hardware/siglent-sds/protocol.c"], INDEPENDENT)
