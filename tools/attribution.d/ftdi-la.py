# The ftdi-la device model (emu/src/models/ftdi-la.rs) and the FTDI shell
# (emu/src/shells/ftdi.rs); see tools/attribution.py.
MAP["emu/src/models/ftdi-la.rs"] = (
    ["libsigrok/src/hardware/ftdi-la/api.c (the chips and the sequence)",
     "libsigrok/src/hardware/ftdi-la/protocol.c (the sample rate and the reads)"], INDEPENDENT)
MAP["emu/src/shells/ftdi.rs"] = (
    ["libftdi1 (ftdi.c), by Intra2net AG and the libftdi developers"],
    "written for oxrok from FTDI's application notes; "
    "libftdi was the reference for which requests a driver sends")
