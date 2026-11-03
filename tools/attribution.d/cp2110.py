# The cp2110 device model (emu/src/models/cp2110.rs); see tools/attribution.py.
MAP["emu/src/models/cp2110.rs"] = (
    ["libsigrok/src/serial_hid_cp2110.c (the reports the chip is sent)",
     "libsigrok/src/serial_hid.c (hidapi calls behind them)",
     "libsigrok/src/dmm/es519xx.c (the UT61E packet layout)"], INDEPENDENT)
