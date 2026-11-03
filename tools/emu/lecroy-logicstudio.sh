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
# before the acquisition (sigrok-cli session.c:826-878). The CLI cases
# therefore cover the scan, the FX2 upload and renumeration, the open,
# the options and those refusals. The capture side (the bitstream, the
# trigger, the status packets, the sample memory) has library-verified
# cases instead (emu_case --libsigrok, named "lib ..."): libsigrok-run
# (tools/libsigrok-run.c) starts the session without a limit, the
# driver ends it after one capture, and the samples go through
# libsigrok's bits output module.
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

# ls_bitstream missing|short|good: takes the 16 channel bitstream
# stand-in away, writes one a byte short, or writes the good one back
# (tools/emu-firmware.sh rewrites what differs). Nothing in --no-oracle's
# dry count of the cases.
ls_bitstream() {
    [ "$EMU_DRY" = 0 ] || return 0
    f=$SIGROK_FIRMWARE_DIR/lecroy-logicstudio16-16.bitstream
    case "$1" in
        missing) rm -f "$f" ;;
        short) sh "$TOOLS/emu-firmware.sh" "$EMU_FIRMWARE_DIR" >/dev/null && truncate -s -1 "$f" ;;
        good) sh "$TOOLS/emu-firmware.sh" "$EMU_FIRMWARE_DIR" >/dev/null ;;
    esac || emu_fail "lecroy-logicstudio: could not make the bitstream stand-in $1"
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

    # Library-verified captures: libsigrok-run in sigrok-cli's place
    # (see the head of this file), on the flashed board, so that no
    # LIBUSB_ERROR_ACCESS retry of a renumerated device reaches stderr.
    # The model's inputs count the samples (input c is bit c of the
    # sample number). With 16 channels a capture is 20000 samples, with
    # 8 it is 40000; the goldens keep the first line of each channel,
    # the trigger marker line of the bits output with its line number
    # before it (so the trigger's place in the capture), and the digest
    # of the whole output.
    T16='1,/^D15:/b; /^T:/{=;b;}; d'
    T8L='1,/^D7:/b; /^T:/{=;b;}; d'
    LO=D0,D1,D2,D3,D4,D5,D6,D7
    HI=D8,D9,D10,D11,D12,D13,D14,D15
    # A failure: libsigrok's log lines and the runner's own, the board's
    # port path made generic.
    P='s#usb/[0-9][0-9.-]*#usb/PORT#'
    ls_options flashed=1
    # The rate classes of lls_set_samplerate() (samplerates[]: the
    # trigger latency differs between them) with 16 channels.
    for r in 1G 500M 250M 100M 10M 1M 1k; do
        emu_case "lib 16ch $r D3 rising" --libsigrok --digest --filter "$T16" -- -d $L -c samplerate=$r -t D3=r -O bits
    done
    emu_case "lib 16ch 100M ratio 30 D4 falling" --libsigrok --digest --filter "$T16" -- -d $L -c samplerate=100M:captureratio=30 -t D4=f -O bits
    # Capture ratios at the ends: the trigger first, and after the last
    # sample (no marker).
    emu_case "lib 16ch ratio 0" --libsigrok --digest --filter "$T16" -- -d $L -c samplerate=1M:captureratio=0 -t D3=r -O bits
    emu_case "lib 16ch ratio 100" --libsigrok --digest --filter "$T16" -- -d $L -c samplerate=1M:captureratio=100 -t D3=r -O bits
    # D13 rises at sample 8192 x 3: the 20000 samples before and after
    # it go round the 20480 sample memory.
    emu_case "lib 16ch memory wraps" --libsigrok --digest --filter "$T16" -- -d $L -c samplerate=1M -t D13=r -O bits
    # Two stages (D0 high, then D0 rising), a level with an edge in one
    # stage, and several levels with an edge.
    emu_case "lib two stages" --libsigrok --digest --filter "$T16" -- -d $L -c samplerate=1M -t D0=1r -O bits
    emu_case "lib D15 level and D0 edge" --libsigrok --digest --filter "$T16" -- -d $L -c samplerate=1M -t D15=1,D0=e -O bits
    emu_case "lib levels and an edge" --libsigrok --digest --filter "$T16" -- -d $L -c samplerate=1M -t D2=1,D4=0,D1=e -O bits
    # Three stages: the driver has two (transform_trigger(),
    # protocol.c:604-606), and the session does not start.
    emu_case "lib three stages" --libsigrok --expect-fail --stderr --filter "$P" -- -d $L -c samplerate=1M -t D0=rfr -O bits
    # 8 channels: the lower group, and the upper one (register 0x21),
    # whose memory wraps.
    emu_case "lib 8ch lower 1M" --libsigrok --digest --filter "$T8L" -- -d $L -C $LO -c samplerate=1M -t D3=r -O bits
    emu_case "lib 8ch lower 500M" --libsigrok --digest --filter "$T8L" -- -d $L -C $LO -c samplerate=500M -t D3=r -O bits
    # Where the part of the memory before the wrap is a whole number of
    # send_samples()'s 4096 sample chunks, the 8 channel path sends the
    # remainder all the same (protocol.c:980-989), a logic packet of 0
    # bytes; libsigrok's bits and hex outputs loop to length - unitsize
    # (bits.c:158, hex.c:163), which wraps for 0, and the run dies of
    # SIGSEGV after the samples before it. sigrok-cli's datafeed_in()
    # hands the empty packet on the same way. The ascii output counts
    # samples (ascii.c:219), so it takes the whole capture.
    emu_case "lib 8ch upper wraps bits crash" --libsigrok --expect-fail --digest --filter "$T16" -- -d $L -C $HI -c samplerate=1M -t D14=r -O bits
    emu_case "lib 8ch upper wraps ascii" --libsigrok --digest --filter '1,/^D15:/b; /T/{=;b;}; d' -- -d $L -C $HI -c samplerate=1M -t D14=r -O ascii
    emu_case "lib 8ch upper 1G ratio 20" --libsigrok --digest --filter "$T16" -- -d $L -C $HI -c samplerate=1G:captureratio=20 -t D9=r -O bits
    # The runner's own limit: the session stopped after 1000 samples,
    # once the driver has sent them all (dev_acquisition_stop() then
    # has nothing to stop).
    emu_case "lib stop after 1000" --libsigrok --digest --filter "$T16" -- -d $L -c samplerate=1M -t D3=r --stop-after 1000 -O bits
    # The C driver never ends: with no trigger it arms nothing and waits
    # (protocol.c:1119, 926-933); an unplug during the fetch or while
    # it waits for the trigger leaves it resubmitting or waiting.
    emu_case "lib no trigger hangs" --libsigrok --hangs --timeout 3 --no-output -- -d $L -c samplerate=1M -O bits
    emu_case "lib unplug mid fetch hangs" --libsigrok --hangs --timeout 3 --no-output --unplug in=2:1 -- -d $L -c samplerate=1M -t D3=r -O bits
    emu_case "lib unplug while waiting hangs" --libsigrok --hangs --timeout 3 --no-output --unplug in=1:2 -- -d $L -c samplerate=1M -t D3=r -O bits
    # Faults that end the session before it starts: the Nth register
    # write (the stop, the thresholds' writes before an upload, the
    # trigger, the arm), the upload, a bitstream chunk, the verify, the
    # settle reads and the status write.
    for f in stall=write:1 stall=write:2 stall=write:5 stall=write:8 stall=write:9 stall=write:27 \
             stall=write:28 stall=write:29 stall=write:30 stall=upload stall_bitstream=5 verify=4 \
             stall=verify stall=read:3 stall=read:5 settle=100 stall=status; do
        ls_options flashed=1 $f
        emu_case "lib fault $f" --libsigrok --expect-fail --stderr --filter "$P" -- -d $L -c samplerate=1M -t D3=r -O bits
    done
    # Faults the driver rides out: a slow verify, slow settling, stale
    # and odd status packets, a short first sample transfer.
    for f in verify=3 settle=99 stale=2 odd_state=0x05 bulk_short=1000; do
        ls_options flashed=1 $f
        emu_case "lib fault $f" --libsigrok --digest --filter "$T16" -- -d $L -c samplerate=1M -t D3=r -O bits
    done
    # And those it never gets out of: the last register write (the
    # fetch's), short status packets, a halted sample endpoint, and
    # inputs that never move.
    for f in stall=write:31 intr_short=16 bulk_stall=1 still=1; do
        ls_options flashed=1 $f
        emu_case "lib fault $f hangs" --libsigrok --hangs --timeout 3 --no-output -- -d $L -c samplerate=1M -t D3=r -O bits
    done
    # The bitstream file missing, or of another size than the driver's
    # FPGA_FIRMWARE_SIZE (upload_fpga_bitstream(), protocol.c:433-445).
    ls_options flashed=1
    ls_bitstream missing
    emu_case "lib bitstream missing" --libsigrok --expect-fail --stderr --filter "$P" -- -d $L -c samplerate=1M -t D3=r -O bits
    ls_bitstream short
    emu_case "lib bitstream short" --libsigrok --expect-fail --stderr --filter "$P" -- -d $L -c samplerate=1M -t D3=r -O bits
    ls_bitstream good
    # Without the FX2 firmware: the scan uploads it, the open waits for
    # the renumerated board, then the capture.
    ls_options
    emu_case "lib unflashed capture" --libsigrok --digest --filter "$T16" -- -d $L -c samplerate=1M -t D3=r -O bits
}
