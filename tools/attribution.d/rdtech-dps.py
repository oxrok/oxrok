# The port (src/drivers/rdtech_dps) and the modbus layer it brings
# (src/modbus); see tools/attribution.py.
MAP["src/modbus/mod.rs"] = (
    ["libsigrok/src/modbus/modbus.c",
     "libsigrok/src/modbus/modbus_serial_rtu.c",
     "libsigrok/src/crc.c"], INDEPENDENT)
MAP["src/drivers/rdtech_dps/mod.rs"] = (
    ["libsigrok/src/hardware/rdtech-dps/api.c",
     "libsigrok/src/hardware/rdtech-dps/protocol.c"], INDEPENDENT)
MAP["src/drivers/rdtech_dps/protocol.rs"] = (
    ["libsigrok/src/hardware/rdtech-dps/api.c",
     "libsigrok/src/hardware/rdtech-dps/protocol.c",
     "libsigrok/src/hardware/rdtech-dps/protocol.h"], INDEPENDENT)
