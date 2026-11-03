# The Victor 70C/86C meter cable (emu/src/models/victor.rs). Sourced by
# tools/diffcli.sh --emu; see docs/USB.md, "Writing a device model".
# The meter streams the diffcli.sh --hw FS9922 packet and then a second
# one, in turn, a packet every 50 polls (the default) of the interrupt endpoint.
#
# Every case names the node: the oracle's libsigrok cannot open a HID
# cable it has to search for (see tools/emu/bu86x.sh).
EMU_MODEL=victor
EMU_OPTIONS="-o packet=2b31323334203130000080000d0a,2d30303432203030000040000d0a"
EMU_DRIVER=victor-dmm
EMU_GCOV_SHARED="src/serial.c src/serial_hid.c src/serial_hid_victor.c"
EMU_GCOV=serial-dmm
# oxrok has the driver: its cases must run full.
EMU_FULL=victor-dmm

emu_cases() {
    SEARCH_BUG="libsigrok 0bc24877 converts a searched path twice (serial_hid.c ser_hid_chip_search)"
    V=victor-dmm:conn=hid/victor/raw=$EMU_HIDRAW
    emu_case "victor scan" --keep-header -- -d $V --scan
    emu_case "victor show" --keep-header -- -d $V --show
    emu_case "victor samples 1" -- -d $V --samples 1
    emu_case "victor samples 2" -- -d $V --samples 2
    emu_case "victor samples 3" -- -d $V --samples 3
    emu_case "victor no chip" -- -d victor-dmm:conn=hid/raw=$EMU_HIDRAW --samples 1
    # The forms users type, which sigrok-cli cannot open (see above):
    # oxrok alone, against the goldens.
    emu_case "victor search hid" --oracle-bug "$SEARCH_BUG" -- -d victor-dmm:conn=hid --samples 1
    emu_case "victor default conn" --oracle-bug "$SEARCH_BUG" -- -d victor-dmm --samples 1
}
