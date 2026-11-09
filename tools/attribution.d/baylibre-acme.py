# The port (src/drivers/baylibre_acme); see tools/attribution.py.
MAP["src/drivers/baylibre_acme/mod.rs"] = (
    ["libsigrok/src/hardware/baylibre-acme/api.c",
     "libsigrok/src/hardware/baylibre-acme/protocol.c"], INDEPENDENT)
MAP["src/drivers/baylibre_acme/protocol.rs"] = (
    ["libsigrok/src/hardware/baylibre-acme/protocol.c",
     "libsigrok/src/hardware/baylibre-acme/protocol.h",
     "libsigrok/src/hardware/baylibre-acme/gpio.c",
     "libsigrok/src/hardware/baylibre-acme/gpio.h"], INDEPENDENT)
MAP["src/drivers/baylibre_acme/gpio.rs"] = (
    ["libsigrok/src/hardware/baylibre-acme/gpio.c",
     "libsigrok/src/hardware/baylibre-acme/gpio.h"], INDEPENDENT)
MAP["src/drivers/baylibre_acme/tests.rs"] = (
    ["libsigrok/src/hardware/baylibre-acme/api.c",
     "libsigrok/src/hardware/baylibre-acme/protocol.c"], INDEPENDENT)
