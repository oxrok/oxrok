# The WCH CH9325 cable of a UNI-T UT61E (emu/src/models/ch9325.rs).
# Sourced by tools/diffcli.sh --emu; see docs/USB.md, "Writing a device
# model". The meter streams the diffcli.sh --hw UT61E packet, a second
# one and five bytes of noise ("?????", which leave the next packet
# misaligned in the driver's buffer: its resync), in turn, one every 50
# polls (the default) of the interrupt endpoint once the host has set
# 19200 baud, 7 bits.
#
# Every case names the node: the oracle's libsigrok cannot open a HID
# cable it has to search for (see tools/emu/bu86x.sh).
EMU_MODEL=ch9325
EMU_OPTIONS="-o packet=3131323334353b3030303a300d0a,3132303435363b3030303a300d0a,3f3f3f3f3f"
EMU_DRIVER=uni-t-ut61e-ser
EMU_GCOV_SHARED="src/serial.c src/serial_hid.c src/serial_hid_ch9325.c"
EMU_GCOV=serial-dmm
# oxrok has the driver: its cases must run full.
EMU_FULL=uni-t-ut61e-ser

emu_cases() {
    SEARCH_BUG="libsigrok 0bc24877 converts a searched path twice (serial_hid.c ser_hid_chip_search)"
    U=uni-t-ut61e-ser:conn=hid/ch9325/raw=$EMU_HIDRAW
    emu_case "ut61e scan" --keep-header -- -d $U --scan
    emu_case "ut61e show" --keep-header -- -d $U --show
    emu_case "ut61e samples 1" -- -d $U --samples 1
    emu_case "ut61e samples 2" --record -- -d $U --samples 2
    emu_case "ut61e samples 4" -- -d $U --samples 4
    emu_case "ut61e no chip" -- -d uni-t-ut61e-ser:conn=hid/raw=$EMU_HIDRAW --samples 1
    # The forms users type, which sigrok-cli cannot open (see above):
    # oxrok alone, against the goldens.
    emu_case "ut61e search chip" --oracle-bug "$SEARCH_BUG" -- -d uni-t-ut61e-ser:conn=hid/ch9325 --samples 1
    emu_case "ut61e search sn" --oracle-bug "$SEARCH_BUG" -- -d uni-t-ut61e-ser:conn=hid/sn=HIDCABLE1 --samples 1

    # The meter misbehaving, through the emulator's options. One that
    # never sends: the scan gives up after its 3 s.
    ch9325_options burst=0
    emu_case "ut61e never sends" --keep-header --no-output -- -d $U --scan
    # 48 bytes of noise before the first packet: more than two packets'
    # worth, which the scan warns about ("Packet search dropped a lot of
    # data") and then takes the packet after it.
    N48=3f3f3f3f3f3f3f3f3f3f3f3f3f3f3f3f3f3f3f3f3f3f3f3f3f3f3f3f3f3f3f3f3f3f3f3f3f3f3f3f3f3f3f3f3f3f3f3f
    ch9325_options packet=$N48,3131323334353b3030303a300d0a,3132303435363b3030303a300d0a
    emu_case "ut61e noise before the first packet" -- -d $U --samples 2
    # 128 bytes of noise: the scan's buffer (serial_stream_detect's) is
    # full without a packet, and the scan gives up although one
    # follows (oxrok searched on until its 3 s had passed, and so found
    # it). A packet every 30 polls, three per open (burst=3): the third
    # comes some 0.6 s after the scan gave up and closed, well inside
    # the 3 s, and nothing comes after it.
    N64=$N48$(echo $N48 | cut -c1-32)
    ch9325_options packet=$N64,$N64,3131323334353b3030303a300d0a period=30 burst=3
    emu_case "ut61e noise fills the scan buffer" --keep-header --no-output -- -d $U --scan
    ch9325_options
}

# The model's options for the cases that follow (see
# tools/emu/ftdi-la.sh); none restores EMU_OPTIONS.
ch9325_options() {
    set -- options "$@"
    emu_cmd "$*" >/dev/null || emu_fail "ch9325: the emulator refused '$*'"
}
