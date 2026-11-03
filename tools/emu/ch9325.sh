# The WCH CH9325 cable of a UNI-T UT61E (emu/src/models/ch9325.rs).
# Sourced by tools/diffcli.sh --emu; see docs/USB.md, "Writing a device
# model". The meter streams the diffcli.sh --hw UT61E packet and then a
# second one, in turn, a packet every 50 polls (the default) of the interrupt
# endpoint once the host has set 19200 baud, 7 bits.
#
# Every case names the node: the oracle's libsigrok cannot open a HID
# cable it has to search for (see tools/emu/bu86x.sh).
EMU_MODEL=ch9325
EMU_OPTIONS="-o packet=3131323334353b3030303a300d0a,3132303435363b3030303a300d0a"
EMU_DRIVER=uni-t-ut61e-ser
EMU_GCOV=serial-dmm
# oxrok has the driver: its cases must run full.
EMU_FULL=uni-t-ut61e-ser

emu_cases() {
    U=uni-t-ut61e-ser:conn=hid/ch9325/raw=$EMU_HIDRAW
    emu_case "ut61e scan" --keep-header -- -d $U --scan
    emu_case "ut61e show" --keep-header -- -d $U --show
    emu_case "ut61e samples 1" -- -d $U --samples 1
    emu_case "ut61e samples 2" -- -d $U --samples 2
    emu_case "ut61e samples 4" -- -d $U --samples 4
    emu_case "ut61e no chip" -- -d uni-t-ut61e-ser:conn=hid/raw=$EMU_HIDRAW --samples 1
}
