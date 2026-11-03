# The LeCroy LogicStudio 16 (emu/src/models/lecroy-logicstudio.rs): an
# FX2 without firmware (05ff:a001) that takes it in the scan and
# renumerates as 05ff:a002; the FPGA bitstream, the trigger and the
# capture follow at the first acquisition. Sourced by tools/diffcli.sh
# --emu; see docs/USB.md, "Writing a device model". oxrok has no port
# of the driver: every case is an oracle case.
#
# sigrok-cli cannot start a capture with this driver: it runs a session
# only with --samples, --time, --frames or --continuous, and the driver
# offers none of them (no SR_CONF_LIMIT_SAMPLES, LIMIT_MSEC,
# LIMIT_FRAMES or CONTINUOUS in devopts, api.c:44-48), so each fails
# before the acquisition (sigrok-cli session.c:826-878). The cases
# therefore cover the scan, the FX2 upload and renumeration, the open,
# the options and those refusals; the model's capture side (the
# bitstream, the trigger, the status packets, the sample memory) is
# held by its unit tests and was checked against the C driver through
# the library (docs in the model).
EMU_MODEL=lecroy-logicstudio
EMU_DRIVER=lecroy-logicstudio
EMU_GCOV_SHARED="src/usb.c src/ezusb.c"

# The model's options for the cases that follow (see fx2lafw.sh).
ls_options() {
    set -- options "$@"
    emu_cmd "$*" >/dev/null || emu_fail "lecroy-logicstudio: the emulator refused '$*'"
}

# Claims interface 0 through usbfs as another program would, until
# ls_release (as fx2lafw_hold in fx2lafw.sh).
ls_hold() {
    timeout 120 python3 -I -c '
import fcntl, os, signal, struct, sys
d = "/sys/bus/usb/devices/" + sys.argv[1]
bus, dev = (int(open(d + f).read()) for f in ("/busnum", "/devnum"))
fd = os.open("/dev/bus/usb/%03d/%03d" % (bus, dev), os.O_RDWR)
fcntl.ioctl(fd, 0x8004550f, struct.pack("I", 0))  # USBDEVFS_CLAIMINTERFACE
print("held", flush=True)
signal.pause()
' "$EMU_BUSID" >"$TMP/lecroy-hold" 2>&1 &
    HOLDPID=$!
    i=0
    until grep -q '^held$' "$TMP/lecroy-hold"; do
        i=$((i + 1))
        if [ $i -gt 100 ] || ! kill -0 $HOLDPID 2>/dev/null; then
            emu_fail "lecroy-logicstudio: could not claim the interface: $(cat "$TMP/lecroy-hold")"
            return 1
        fi
        sleep 0.1
    done
}

ls_release() {
    kill $HOLDPID 2>/dev/null
    wait $HOLDPID 2>/dev/null
    HOLDPID=""
}

emu_cases() {
    L=lecroy-logicstudio
    # The driver's and sigrok-cli's own lines, without the open of the
    # renumerated device that libusb may refuse with LIBUSB_ERROR_ACCESS
    # before udev opens its node (docs/USB.md; api.c:226-229 retries).
    E='/LIBUSB_ERROR_ACCESS/d; /^(Failed|This device|sr: |No devices)/!d'

    # Without firmware: every run uploads it in the scan, and dev_open()
    # waits (300 ms, then up to 3 s) for the renumerated device.
    ls_options
    emu_case "scan" --keep-header -- -d $L --scan
    # The driver lists no scan options: config_list() answers
    # SR_CONF_SCAN_OPTIONS with SR_ERR_NA (api.c:411-422), so libsigrok's
    # check_options() fails without a word and sr_driver_scan() returns
    # no device (hwdriver.c:513-516, 583-586): conn= cannot select the
    # board, and the scan it is given finds nothing.
    emu_case "conn refused" --expect-fail --stderr --filter "$E" -- -d $L:conn=$EMU_USB --get samplerate
    emu_case "show" --keep-header -- -d $L --show
    emu_case "get samplerate" -- -d $L --get samplerate
    emu_case "get captureratio" -- -d $L --get captureratio
    emu_case "set and get samplerate" -- -d $L -c samplerate=2500k --get samplerate
    emu_case "set and get captureratio" -- -d $L -c captureratio=0 --get captureratio
    # 1 GHz is in protocol.c's table (lls_set_samplerate()) but not in
    # api.c's list: the driver takes it.
    emu_case "1G off the list" -- -d $L -c samplerate=1G --get samplerate
    emu_case "bad rate" --expect-fail --stderr --filter "$E" -- -d $L -c samplerate=3M --get samplerate
    emu_case "captureratio 101" --expect-fail --stderr --filter "$E" -- -d $L -c captureratio=101 --get captureratio
    emu_case "set captureratio" --no-output -- -d $L --set -c captureratio=20
    emu_case "get conn" --expect-fail --stderr --filter "$E" -- -d $L --get conn
    # No way to run a session (see the head of this file).
    emu_case "samples refused" --expect-fail --stderr --filter "$E" -- -d $L --samples 100 -O hex
    emu_case "time refused" --expect-fail --stderr --filter "$E" -- -d $L --time 10 -O hex
    emu_case "frames refused" --expect-fail --stderr --filter "$E" -- -d $L --frames 1 -O hex
    emu_case "continuous refused" --expect-fail --stderr --filter "$E" -- -d $L --continuous -O hex
    emu_case "trigger and samples refused" --expect-fail --stderr --filter "$E" -- -d $L -t D0=r --samples 20000 -O hex
    emu_case "8 channels refused" --expect-fail --stderr --filter "$E" -- -d $L -C D8,D9,D10,D11,D12,D13,D14,D15 -t D9=f --samples 40000 -O hex

    # The firmware already running: no upload, the open is immediate.
    ls_options flashed=1
    emu_case "flashed scan" --keep-header -- -d $L --scan
    emu_case "flashed show" --keep-header -- -d $L --show
    emu_case "flashed get samplerate" -- -d $L --get samplerate
    emu_case "flashed samples refused" --expect-fail --stderr --filter "$E" -- -d $L --samples 100 -O hex
    if ls_hold; then
        emu_case "interface claimed by another program" --expect-fail --stderr --filter "$E" -- -d $L --get samplerate
        ls_release
    fi

    # A firmware that never comes back (renum=0): dev_open() polls for
    # 3 s of wall time and gives up (api.c:279-296).
    ls_options renum=0
    emu_case "no renumeration" --expect-fail --stderr --filter "$E" -- -d $L --get samplerate

    # Unplugged during the FX2 upload in the scan (the Nth control write:
    # SET_CONFIGURATION 0, the reset hold, the 2 chunks, the release):
    # ezusb_upload_firmware() fails and scan() drops the device
    # (api.c:150-158).
    ls_options
    U='/Firmware upload failed|^No devices/!d'
    emu_case "unplug before the reset hold" --unplug out=0:1 --expect-fail --stderr --filter "$U" -- -d $L --get samplerate
    emu_case "unplug during the upload" --unplug out=0:3 --expect-fail --stderr --filter "$U" -- -d $L --get samplerate
    emu_case "unplug before the release" --unplug out=0:4 --expect-fail --stderr --filter "$U" -- -d $L --get samplerate
}
