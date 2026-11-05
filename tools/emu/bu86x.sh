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
EMU_GCOV_SHARED="src/serial.c src/serial_hid.c src/serial_hid_bu86x.c"
EMU_GCOV=serial-dmm
# oxrok has the driver: its cases must run full.
EMU_FULL=brymen-bm86x

emu_cases() {
    SEARCH_BUG="libsigrok 0bc24877 converts a searched path twice (serial_hid.c ser_hid_chip_search)"
    B=brymen-bm86x:conn=hid/bu86x/raw=$EMU_HIDRAW
    emu_case "bm86x scan" --keep-header -- -d $B --scan
    emu_case "bm86x show" --keep-header -- -d $B --show
    emu_case "bm86x samples 1" -- -d $B --samples 1
    emu_case "bm86x samples 3" --record -- -d $B --samples 3
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

    # The meter misbehaving, through the emulator's options (the two
    # packets, without drop=4, plus the fault). A meter that never
    # answers: the scan gives up after its 3 s.
    P=packet=000001a0dbf8e47c01007cbebf0004008686868600000000,000001a0dbbee47c01007cbebf0004008686868600000000
    bu86x_options $P drop=1
    emu_case "bm86x never answers" --keep-header --no-output -- -d $B --scan
    # Two answers lost in a row: the driver asks again 500 ms after
    # each, and the third request of the acquisition is answered.
    bu86x_options $P lose=2,3
    emu_case "bm86x two lost in a row" -- -d $B --samples 2
    # The cable STALLs the acquisition's first request: hid_write fails,
    # sr_brymen_bm86x_packet_request ignores it, and the driver asks
    # again after 500 ms.
    bu86x_options $P stall=2
    emu_case "bm86x request stalled" -- -d $B --samples 2
    # The scan's request STALLed: no answer, no device, no error.
    bu86x_options $P stall=1
    emu_case "bm86x scan request stalled" --keep-header --no-output -- -d $B --scan

    # A BM52x behind the cable (meter=bm52x: 12.34 V DC, 50.00 Hz),
    # whose request, unlike the BM86x's, fails when its write does.
    B52=brymen-bm52x:conn=hid/bu86x/raw=$EMU_HIDRAW
    bu86x_options meter=bm52x
    emu_case "bm52x scan" --keep-header -- -d $B52 --scan
    emu_case "bm52x samples 2" -- -d $B52 --samples 2
    # The scan's request STALLed: "Failed to request packet", no device.
    bu86x_options meter=bm52x stall=1
    emu_case "bm52x scan request stalled" --keep-header --no-output -- -d $B52 --scan
    # The acquisition's second request STALLed: receive_data's timeout
    # branch fails, its source goes, and the session ends after the
    # first sample.
    bu86x_options meter=bm52x stall=3
    emu_case "bm52x request stalled" --oxrok-fails "hid: Broken pipe (os error 32)" -- -d $B52 --samples 3
    bu86x_options
}

# The model's options for the cases that follow (see
# tools/emu/ftdi-la.sh); none restores EMU_OPTIONS.
bu86x_options() {
    set -- options "$@"
    emu_cmd "$*" >/dev/null || emu_fail "bu86x: the emulator refused '$*'"
}
