# The LeCroy LogicStudio 16 (emu/src/models/lecroy-logicstudio.rs): an
# FX2 without firmware (05ff:a001) that takes it in the scan and
# renumerates as 05ff:a002; the FPGA bitstream, the trigger and the
# capture follow at the first acquisition. Sourced by tools/diffcli.sh
# --emu; see docs/USB.md, "Writing a device model".
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
#
# oxrok's port (src/drivers/lecroy_logicstudio) starts a capture with -O,
# -o or -t alone, so a capture takes the same arguments under both. A "lib"
# case with --oxrok is a full case against the runner: oxrok is held to its
# goldens, output and transactions, and --no-oracle runs oxrok against them.
# Where the C driver cannot be the oracle (it hangs or dies), an "oxrok"
# case keeps the runner's failure in SLUG.oracle and pins oxrok's own output
# and transactions in SLUG.out and SLUG.tx; its "lib" case stays, library
# only, for what the C driver does. No --oracle-differs: nothing here has
# the C driver ending well with another result.
EMU_MODEL=lecroy-logicstudio
EMU_DRIVER=lecroy-logicstudio
EMU_FULL=lecroy-logicstudio
EMU_GCOV_SHARED="src/usb.c src/ezusb.c"

# The model's options for the cases that follow (see fx2lafw.sh).
ls_options() {
    set -- options "$@"
    emu_cmd "$*" >/dev/null || emu_fail "lecroy-logicstudio: the emulator refused '$*'"
}

# Claims interface 0 through usbfs as another program would, until
# ls_release (as fx2lafw_hold in fx2lafw.sh).
ls_hold() {
    [ "$EMU_DRY" = 0 ] || return 0
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
    [ "$EMU_DRY" = 0 ] || return 0
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
    #
    # X names oxrok's log lines as libsigrok names them, and gives its last
    # lines sigrok-cli's words: the reason off "Failed to open device" (the
    # driver's own line says it), and sr_strerror()'s words for the reason
    # of a refused option, whose oxrok text names the rate or the range. No
    # line goes. sigrok-cli's and the runner's lines do not match.
    D='oxrok: err: oxrok::drivers::lecroy_logicstudio[a-z_:]*:'
    X="s/^$D (Capture ratio)/sr: hwdriver: \\1/; s/^$D /sr: lecroy-logicstudio: /; s/^oxrok: err: oxrok::cli::run: /sr: hwdriver: /; s/^(Failed to open device): .*\$/\\1./; s/^(Failed to set device option 'samplerate'): invalid argument: .*\$/\\1: generic\\/unspecified error./; s/^(Failed to set device option 'captureratio': invalid argument): .*\$/\\1./"
    E="$X; /LIBUSB_ERROR_ACCESS/d; /^(Failed|This device|sr: |No devices)/!d"

    # Without firmware: every run uploads it in the scan, and dev_open()
    # waits (300 ms, then up to 3 s) for the renumerated device.
    ls_options
    emu_case "scan" --keep-header -- -d $L --scan
    # The driver lists no scan options: config_list() answers
    # SR_CONF_SCAN_OPTIONS with SR_ERR_NA (api.c:411-422), so libsigrok's
    # check_options() fails without a word and sr_driver_scan() returns
    # no device (hwdriver.c:513-516, 583-586): conn= cannot select the
    # board, and the scan it is given finds nothing.
    # oxrok says which option it refuses, and pins that.
    emu_case "conn refused" --oracle-bug "check_options() refuses a scan option without a word and the scan finds no device, hwdriver.c:513-516, 583-586" --expect-fail --stderr --filter "$X" -- -d $L:conn=$EMU_USB --get samplerate
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
    # A start that fails. libsigrok's core lines ("sr: session: ..." and
    # "sr: resource: ...") have no counterpart: oxrok's session hands back
    # the driver's error, and the last line of either tool is reduced to
    # where it failed, as in ikalogic-scanaplus, pipistrello-ols and
    # saleae-logic16. The driver's own lines are compared.
    F="/^sr: (session|resource): /d; $X; \$ { /^sr: lecroy-logicstudio: /! s/.*/start refused/; }"
    # oxrok's run alone, its lines named as above; nothing goes.
    P="$X"
    ls_options flashed=1
    # The runner's --scan against sigrok-cli's: this golden must equal
    # flashed-scan.out.
    emu_case "lib flashed scan" --libsigrok --oxrok --keep-header -- -d $L --scan
    # The rate classes of lls_set_samplerate() (samplerates[]: the
    # trigger latency differs between them) with 16 channels.
    for r in 1G 500M 250M 100M 10M 1M 1k; do
        emu_case "lib 16ch $r D3 rising" --libsigrok --oxrok --digest --filter "$T16" -- -d $L -c samplerate=$r -t D3=r -O bits
    done
    emu_case "lib 16ch 100M ratio 30 D4 falling" --libsigrok --oxrok --digest --filter "$T16" -- -d $L -c samplerate=100M:captureratio=30 -t D4=f -O bits
    # Capture ratios at the ends: the trigger first, and after the last
    # sample (no marker).
    emu_case "lib 16ch ratio 0" --libsigrok --oxrok --digest --filter "$T16" -- -d $L -c samplerate=1M:captureratio=0 -t D3=r -O bits
    emu_case "lib 16ch ratio 100" --libsigrok --oxrok --digest --filter "$T16" -- -d $L -c samplerate=1M:captureratio=100 -t D3=r -O bits
    # D13's first rise once armed is at sample 24576, so the capture
    # goes round the 20480 sample memory.
    emu_case "lib 16ch memory wraps" --libsigrok --oxrok --digest --filter "$T16" -- -d $L -c samplerate=1M -t D13=r -O bits
    # Two stages (D0 high, then D0 rising), a level with an edge in one
    # stage, and several levels with an edge.
    emu_case "lib two stages" --libsigrok --oxrok --digest --filter "$T16" -- -d $L -c samplerate=1M -t D0=1r -O bits
    emu_case "lib D15 level and D0 edge" --libsigrok --oxrok --digest --filter "$T16" -- -d $L -c samplerate=1M -t D15=1,D0=e -O bits
    emu_case "lib levels and an edge" --libsigrok --oxrok --digest --filter "$T16" -- -d $L -c samplerate=1M -t D2=1,D4=0,D1=e -O bits
    # Three stages: the driver has two (transform_trigger(),
    # protocol.c:604-606), and the session does not start.
    emu_case "lib three stages" --libsigrok --oxrok --expect-fail --stderr --filter "$F" -- -d $L -c samplerate=1M -t D0=rfr -O bits
    # 8 channels: the lower group, and the upper one (register 0x21),
    # whose memory wraps.
    emu_case "lib 8ch lower 1M" --libsigrok --oxrok --digest --filter "$T8L" -- -d $L -C $LO -c samplerate=1M -t D3=r -O bits
    emu_case "lib 8ch lower 500M" --libsigrok --oxrok --digest --filter "$T8L" -- -d $L -C $LO -c samplerate=500M -t D3=r -O bits
    # send_samples() sends a logic packet of 0 bytes for any stretch of
    # a whole number of its 4096 sample chunks in the 8 channel path
    # (protocol.c:980-989; 8192 bytes here, after the wrap, before the
    # trigger) and for an empty stretch in the 16 channel path
    # (protocol.c:957-960, from 1062 when the trigger is where the read
    # starts). libsigrok's outputs do not take it: bits, hex and ols loop
    # to length - unitsize (bits.c:158, hex.c:163, ols.c:121), which
    # wraps for 0, and die of SIGSEGV; csv dies of SIGFPE. sigrok-cli's
    # datafeed_in() hands the empty packet on the same way. The ascii
    # output counts samples (ascii.c:219), so it takes the whole capture.
    emu_case "lib 8ch upper wraps bits crash" --libsigrok --expect-fail --digest --filter "$T16" -- -d $L -C $HI -c samplerate=1M -t D14=r -O bits
    emu_case "lib 8ch upper wraps ascii" --libsigrok --oxrok --digest --filter '1,/^D15:/b; /T/{=;b;}; d' -- -d $L -C $HI -c samplerate=1M -t D14=r -O ascii
    emu_case "lib 8ch upper 1G ratio 20" --libsigrok --oxrok --digest --filter "$T16" -- -d $L -C $HI -c samplerate=1G:captureratio=20 -t D9=r -O bits
    # The runner's own limit (--stop-after): the output cut at 1000
    # samples. The driver sends the samples and ends the acquisition in
    # one callback (recv_bulk_transfer()), so the stop asked from the
    # main loop comes after that end: the capture is over, its write to
    # the arm register goes out as the last transaction, and stderr stays
    # empty of the failure a stop nested in that callback would log.
    emu_case "lib stop after 1000" --libsigrok --digest --filter "$T16" -- -d $L -c samplerate=1M -t D3=r --stop-after 1000 -O bits
    emu_case "lib stop after 1000 stderr" --libsigrok --no-output --stderr -- -d $L -c samplerate=1M -t D3=r --stop-after 1000 -O bits
    # The C driver never ends: with no trigger it arms nothing and waits
    # (protocol.c:1119, 926-933); an unplug during the fetch or while
    # it waits for the trigger leaves it resubmitting or waiting.
    emu_case "lib no trigger hangs" --libsigrok --hangs --timeout 3 --no-output -- -d $L -c samplerate=1M -O bits
    emu_case "lib unplug mid fetch hangs" --libsigrok --hangs --timeout 3 --no-output --unplug in=2:1 -- -d $L -c samplerate=1M -t D3=r -O bits
    emu_case "lib unplug while waiting hangs" --libsigrok --hangs --timeout 3 --no-output --unplug in=1:2 -- -d $L -c samplerate=1M -t D3=r -O bits
    # Where the C driver never ends, oxrok does, or refuses; --oracle-hangs
    # holds the runner to its hang. The errors are oxrok's own lines.
    H='recv_intr_transfer() submits no transfer after a failure or a short packet, protocol.c:926-933'
    emu_case "oxrok no trigger" --libsigrok --oxrok --oracle-bug "without a trigger nothing arms the board and nothing reads its packets, protocol.c:331,1119" --oracle-hangs --timeout 3 --expect-fail --stderr --filter "$P" -- -d $L -c samplerate=1M -O bits
    emu_case "oxrok unplug mid fetch" --libsigrok --oxrok --oracle-bug "recv_bulk_transfer() takes no status, and its resubmit to the gone device fails unchecked, protocol.c:1017-1026, so nothing more completes" --oracle-hangs --timeout 3 --stderr --filter "$P" --unplug in=2:1 -- -d $L -c samplerate=1M -t D3=r -O bits
    emu_case "oxrok unplug while waiting" --libsigrok --oxrok --oracle-bug "$H" --oracle-hangs --timeout 3 --stderr --filter "$P" --unplug in=1:2 -- -d $L -c samplerate=1M -t D3=r -O bits
    # The same capture with bits as "lib 8ch upper wraps ascii", which the
    # C driver's packet of no bytes crashes.
    emu_case "oxrok 8ch upper wraps bits" --libsigrok --oxrok --oracle-bug "send_samples() sends a logic packet of 0 bytes, protocol.c:980-989, and bits.c:158 wraps" --digest --filter "$T16" -- -d $L -C $HI -c samplerate=1M -t D14=r -O bits
    # Faults that end the session before it starts: the Nth register
    # write (1 the stop, 2 and 5 the thresholds, 8 the channel group,
    # 9 the trigger, up to the arm at 30), the upload, a bitstream
    # chunk, the verify, the settle reads (the first pass and a later
    # one) and the status write.
    for f in stall=write:1 stall=write:2 stall=write:5 stall=write:8 stall=write:9 stall=write:27 \
             stall=write:28 stall=write:29 stall=write:30 stall_bitstream=5 verify=4 \
             stall=verify stall=read:3 stall=read:5 settle=100 stall=status; do
        ls_options flashed=1 $f
        emu_case "lib fault $f" --libsigrok --oxrok --expect-fail --stderr --filter "$F" -- -d $L -c samplerate=1M -t D3=r -O bits
    done
    # A refused 0xb3: oxrok names the STALL, LIBUSB_ERROR_PIPE, where the C
    # reads LIBUSB_ERROR_IO whatever happened (kept in the .oracle).
    ls_options flashed=1 stall=upload
    emu_case "lib fault stall=upload" --libsigrok --oxrok --oracle-bug "names ret, not r, protocol.c:452-454" --expect-fail --stderr --filter "$F" -- -d $L -c samplerate=1M -t D3=r -O bits
    # Faults the driver rides out: a slow verify, slow settling, stale
    # and odd status packets, a short first sample transfer.
    for f in verify=3 settle=99 stale=2 odd_state=0x05 bulk_short=1000; do
        ls_options flashed=1 $f
        emu_case "lib fault $f" --libsigrok --oxrok --digest --filter "$T16" -- -d $L -c samplerate=1M -t D3=r -O bits
    done
    # And those it never gets out of: the last register write (the
    # fetch's), short status packets, a halted sample endpoint, and
    # inputs that never move.
    for f in stall=write:31 intr_short=16 bulk_stall=1 still=1; do
        ls_options flashed=1 $f
        emu_case "lib fault $f hangs" --libsigrok --hangs --timeout 3 --no-output -- -d $L -c samplerate=1M -t D3=r -O bits
    done
    # The same under oxrok, each pinned to its own end: the failed fetch
    # write, the short packet, the halted endpoint.
    ls_options flashed=1 stall=write:31
    emu_case "oxrok fault stall=write:31" --libsigrok --oxrok --oracle-bug "handle_fetch_samples_done() ignores the status of its write, protocol.c:244-264, and resubmits for ever" --oracle-hangs --timeout 3 --stderr --filter "$P" -- -d $L -c samplerate=1M -t D3=r -O bits
    ls_options flashed=1 intr_short=16
    emu_case "oxrok fault intr_short=16" --libsigrok --oxrok --oracle-bug "$H" --oracle-hangs --timeout 3 --stderr --filter "$P" -- -d $L -c samplerate=1M -t D3=r -O bits
    ls_options flashed=1 bulk_stall=1
    emu_case "oxrok fault bulk_stall=1" --libsigrok --oxrok --oracle-bug "recv_bulk_transfer() resubmits whatever the status, protocol.c:1017-1027" --oracle-hangs --timeout 3 --stderr --filter "$P" -- -d $L -c samplerate=1M -t D3=r -O bits
    # Ctrl-C. Neither sigrok-cli nor the runner has a SIGINT handler, and
    # a trigger that does not fire is waited for until then; the stop
    # writes the arm register (as "lib stop while waiting"), on a board
    # that told its state before, and on one that says nothing, right after
    # the arm (the 32nd control write here, bitstream and FX2 requests
    # included). The latter's transactions are "lib stop while waiting"'s
    # with the harness's interrupt line among them.
    I="the runner has no SIGINT handler, like sigrok-cli"
    ls_options flashed=1 still=1
    emu_case "oxrok interrupt while waiting" --libsigrok --oxrok --oracle-bug "$I" --interrupt in=1:2 --no-output -- -d $L -c samplerate=1M -t D3=r -O bits
    ls_options flashed=1 intr_silent=1
    emu_case "oxrok interrupt on a silent board" --libsigrok --oxrok --oracle-bug "$I" --interrupt out=0:32 --no-output -- -d $L -c samplerate=1M -t D3=r -O bits
    # A stop while the trigger wait is pending: --stop-after 0 asks for
    # it as the session starts. How many status packets complete before
    # the main loop gets to it depends on timing, so a board that sends
    # none. The stop writes the arm register, and the interrupt transfer's
    # timeout finds the abort (recv_intr_transfer(), protocol.c:920-923).
    ls_options flashed=1 intr_silent=1
    emu_case "lib stop while waiting" --libsigrok --no-output -- -d $L -c samplerate=1M -t D3=r --stop-after 0 -O bits
    # The bitstream file missing, or of another size than the driver's
    # FPGA_FIRMWARE_SIZE (upload_fpga_bitstream(), protocol.c:433-445).
    ls_options flashed=1
    ls_bitstream missing
    emu_case "lib bitstream missing" --libsigrok --oxrok --expect-fail --stderr --filter "$F" -- -d $L -c samplerate=1M -t D3=r -O bits
    ls_bitstream short
    emu_case "lib bitstream short" --libsigrok --oxrok --expect-fail --stderr --filter "$F" -- -d $L -c samplerate=1M -t D3=r -O bits
    ls_bitstream good
    # Replay fixtures: a capture with each bitstream, of few channels to
    # keep their output small.
    ls_options flashed=1
    emu_case "lib recorded 16ch" --libsigrok --oxrok --record -- -d $L -C D3,D8 -c samplerate=1M -t D3=r -O bits
    emu_case "lib recorded 8ch upper" --libsigrok --oxrok --record -- -d $L -C D9 -c samplerate=100M:captureratio=20 -t D9=f -O bits
    # Without the FX2 firmware: the scan uploads it, the open waits for
    # the renumerated board, then the capture.
    ls_options
    emu_case "lib unflashed capture" --libsigrok --oxrok --digest --filter "$T16" -- -d $L -c samplerate=1M -t D3=r -O bits
}
