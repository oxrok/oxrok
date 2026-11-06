# A UNI-T UT181A behind its CP2110 cable (emu/src/models/uni-t-ut181a-cp2110.rs):
# the driver's frames through serial_hid_cp2110.c and oxrok's serial::hid, a few
# of each path. tools/emu/uni-t-ut181a.sh has the driver's own cases. Sourced by
# tools/diffcli.sh --emu; each case names the node, as the oracle cannot open a
# HID cable it has to search for (see tools/emu/bu86x.sh).
EMU_MODEL=uni-t-ut181a-cp2110
EMU_DRIVER=uni-t-ut181a
EMU_FULL=$EMU_DRIVER
EMU_GCOV=uni-t-ut181a
EMU_GCOV_SHARED="src/serial.c src/serial_hid.c src/serial_hid_cp2110.c"

emu_cases() {
    U=uni-t-ut181a:conn=hid/cp2110/raw=$EMU_HIDRAW
    # cp2110_write_bytes sends a 64 byte report of which it filled the count and the
    # bytes (serial_hid_cp2110.c:246,263-265): the rest is what the stack held. The
    # meter takes the count's bytes, so the tail goes from the comparison, and
    # oxrok's zeros with it. The lengths are the commands': 7, 8, 9 and 13 bytes.
    T='s/^(int_out 01 07[0-9a-f]{14})[0-9a-f]*/\1/
s/^(int_out 01 08[0-9a-f]{16})[0-9a-f]*/\1/
s/^(int_out 01 09[0-9a-f]{18})[0-9a-f]*/\1/
s/^(int_out 01 0d[0-9a-f]{26})[0-9a-f]*/\1/'
    emu_options setters=0
    emu_case "scan" --tx-filter "$T" --keep-header -- -d $U --scan
    emu_case "get range" --tx-filter "$T" -- -d $U --get range
    emu_options
    emu_case "set range" --tx-filter "$T" --no-output -- -d $U -c range=6V --set
    # Bounded by frames, and long periods: a poll here is about 1 ms.
    emu_options setters=0 period=100 frames=2
    emu_case "samples" --tx-filter "$T" -- -d $U --samples 2 -O analog
    # A record info and a chunk across reports, with the padding the oracle's build
    # needs (tools/emu/uni-t-ut181a.sh).
    emu_options setters=0 pad=300 per=8 rec=One/VDC/2/12
    emu_case "recording" --tx-filter "$T" -- -d $U -c data_source=One --samples 100 -O analog
    emu_options setters=0 saved=na@3111.1:m=1.5/30/VDC,na@3111.1:m=2.5/30/VDC
    emu_case "save" --tx-filter "$T" -- -d $U -c data_source=Save --samples 100 -O analog
    emu_options
}
