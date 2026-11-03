# The Brymen BU-86X cable of a BM86x meter (emu/src/models/bu86x.rs).
# Sourced by tools/diffcli.sh --emu; see docs/USB.md, "Writing a device
# model". The meter answers the diffcli.sh --hw packet (1.2345 V AC,
# 50.0 Hz) and then a second one (1.2045 V AC), in turn; the scan takes
# the first answer. Every 4th request goes unanswered (drop=4), so
# "bm86x samples 3" sees the driver ask again after 500 ms.
#
# Every case names the node: the oracle's libsigrok (0bc24877) cannot
# open a HID cable it has to search for (hid, hid/CHIP, hid/CHIP/sn=):
# ser_hid_chip_search converts the found port name to a path a second
# time, "raw=hid/bu86x/raw=/dev/hidrawN", and hid_open_path fails.
EMU_MODEL=bu86x
EMU_OPTIONS="-o packet=000001a0dbf8e47c01007cbebf0004008686868600000000,000001a0dbbee47c01007cbebf0004008686868600000000 -o drop=4"
EMU_DRIVER=brymen-bm86x
EMU_GCOV=serial-dmm
# oxrok has the driver: its cases must run full.
EMU_FULL=brymen-bm86x

emu_cases() {
    SEARCH_BUG="libsigrok 0bc24877 converts a searched path twice (serial_hid.c ser_hid_chip_search)"
    B=brymen-bm86x:conn=hid/bu86x/raw=$EMU_HIDRAW
    emu_case "bm86x scan" --keep-header -- -d $B --scan
    emu_case "bm86x show" --keep-header -- -d $B --show
    emu_case "bm86x samples 1" -- -d $B --samples 1
    emu_case "bm86x samples 3" -- -d $B --samples 3
    emu_case "bm86x P2 only" -- -d $B -C P2 --samples 2
    emu_case "bm86x csv" -- -d $B --samples 2 -O csv:header=false
    # The chip from the node's USB id.
    emu_case "bm86x no chip" -- -d brymen-bm86x:conn=hid/raw=$EMU_HIDRAW --samples 1
    # The forms users type, which sigrok-cli cannot open (see above):
    # oxrok alone, against the goldens.
    emu_case "bm86x search hid" --oracle-bug "$SEARCH_BUG" -- -d brymen-bm86x:conn=hid --samples 1
    emu_case "bm86x search chip" --oracle-bug "$SEARCH_BUG" -- -d brymen-bm86x:conn=hid/bu86x --samples 1
    emu_case "bm86x search sn" --oracle-bug "$SEARCH_BUG" -- -d brymen-bm86x:conn=hid/bu86x/sn=HIDCABLE1 --samples 1
    emu_case "bm86x default conn" --oracle-bug "$SEARCH_BUG" -- -d brymen-bm86x --samples 1
}
