# A Testo 435 (emu/src/models/testo.rs), an MCU behind an FT232R that the
# driver reaches with plain libusb. Sourced by tools/diffcli.sh --emu; see
# docs/USB.md, "Writing a device model".
#
# Stock libsigrok finds no Testo at all: b258c09f dropped the sdi->driver
# assignment that sr_dev_open() (device.c:730) needs during the scan. Every
# golden here comes from the oracle with that one line restored
# (tools/oracle-patches/libsigrok-testo-sdi-driver.patch).
#
# Lines of the C no case reaches (gcov.txt): api.c's scan 68 (no other scan
# option exists) and 113 (a port path always exists here); dev_open 153,
# 158-160 (a libusb open or a detach that fails); dev_close 180; config_get
# 198-203, 207-211 (sr_config_get() refuses keys outside devopts first);
# receive_transfer 299-303, dev_acquisition_start 370, 377-380 and
# protocol.c's 160-162 (a submit that fails: an unplug loses that race, the
# transfer goes and then reports no device); protocol.c 177, 194 (callers never
# pass so little).
EMU_MODEL=testo
EMU_DRIVER=testo
# oxrok has the driver (src/drivers/testo): every case runs both tools.
EMU_FULL=testo
EMU_GCOV_SHARED="src/usb.c"

# The model's options for the cases that follow; none restores the default.
testo_options() {
    set -- options "$@"
    emu_cmd "$*" >/dev/null || emu_fail "testo: the emulator refused '$*'"
}

# Claims interface 0 through usbfs as another program would, until
# testo_release (as logicpro_hold in saleae-logic-pro.sh).
testo_hold() {
    case "testo interface claimed" in *"$filter"*) ;; *) return 1 ;; esac # no such case in a -k run
    [ "$EMU_DRY" = 0 ] || return 0 # diffcli counting cases: no device, nothing to claim
    timeout 120 python3 -I -c '
import fcntl, os, signal, struct, sys
d = "/sys/bus/usb/devices/" + sys.argv[1]
bus, dev = (int(open(d + f).read()) for f in ("/busnum", "/devnum"))
fd = os.open("/dev/bus/usb/%03d/%03d" % (bus, dev), os.O_RDWR)
fcntl.ioctl(fd, 0x8004550f, struct.pack("I", 0))  # USBDEVFS_CLAIMINTERFACE
print("held", flush=True)
signal.pause()
' "$EMU_BUSID" >"$TMP/testo-hold" 2>&1 &
    HOLDPID=$!
    i=0
    until grep -q '^held$' "$TMP/testo-hold"; do
        i=$((i + 1))
        if [ $i -gt 100 ] || ! kill -0 $HOLDPID 2>/dev/null; then
            emu_fail "testo: could not claim the interface: $(cat "$TMP/testo-hold")"
            return 1
        fi
        sleep 0.1
    done
}

testo_release() {
    kill $HOLDPID 2>/dev/null
    wait $HOLDPID 2>/dev/null
    HOLDPID=""
}

emu_cases() {
    D=testo
    # oxrok's log prefix as sigrok-cli's. Nothing else: the messages stay.
    E='s/^oxrok: [a-z]+: oxrok::drivers::testo(::protocol)?: /sr: testo: /'
    # A start that fails: session.c:831-835's line is libsigrok's own, and each
    # tool ends on its own last line; the driver's line says what failed.
    S="$E; /^sr: session: /d; s/^(Failed to start session\\.|device error: .*)\$/start refused/"
    emu_case "testo scan" --keep-header -- -d $D --scan
    emu_case "testo show" --keep-header -- -d $D --show
    emu_case "testo get limit_samples" -- -d $D --get limit_samples
    emu_case "testo get limit_time" -- -d $D --get limit_time
    # devopts (api.c:34-38) lacks conn, though config_get() has it (api.c:197).
    emu_case "testo get conn" --expect-fail --stderr -- -d $D --get conn
    emu_case "testo set limit_samples" --no-output -- -d $D --set -c limit_samples=5
    emu_case "testo one sample" -- -d $D --samples 1
    emu_case "testo five samples" --record -- -d $D --samples 5
    # sigrok-cli stops a --continuous at the EOF on its stdin: the runner.
    emu_case "testo limit by option" --libsigrok --oxrok -- -d $D -c limit_samples=2 --continuous
    emu_case "testo csv" --filter '/^; (CSV|from)/d' -- -d $D --samples 3 -O csv
    emu_case "testo one channel" -- -d $D --samples 3 -C Humidity
    emu_case "testo by conn" -- -d $D:conn=$EMU_USB --samples 2
    W="api.c:75 filters only if (conn_devices): a conn sr_usb_find() matches nothing to selects every device"
    emu_case "testo conn matches nothing" --oracle-differs "$W" --keep-header --no-output -- -d $D:conn=99.99 --scan
    # The driver's own protocol steps, not the core's lines.
    LOG="$E; /testo: (Probing|Found|Requested|Got|Packet)/!d;s/^.*testo: /testo: /"
    emu_case "testo debug log" --stderr --filter "$LOG" -- -d $D -l 5 --samples 2

    testo_options units=1,3,5,24
    emu_case "testo four units" -- -d $D --samples 3
    testo_options units=
    emu_case "testo no units" --keep-header -- -d $D --scan
    # Two of a unit: two channels of one name.
    testo_options units=1,1
    emu_case "testo same unit twice scan" --keep-header -- -d $D --scan
    testo_options units=1 values=-12.5
    emu_case "testo negative reading" -- -d $D --samples 1
    testo_options units=1 values=inf
    emu_case "testo infinite reading" -- -d $D --samples 1
    # printf's nan, in oxrok's analog output too.
    testo_options units=1 values=nan
    emu_case "testo nan reading" -- -d $D --samples 1

    # The answer in pieces, and late.
    testo_options chunk=3
    emu_case "testo answer in pieces" --record -- -d $D --samples 3
    testo_options delay=4
    emu_case "testo answer late" -- -d $D --samples 3
    testo_options stale=10
    emu_case "testo flush" -- -d $D --samples 1
    testo_options tail=6
    emu_case "testo tail of an old answer" -- -d $D --samples 1
    testo_options

    # Not this driver's device.
    testo_options manufacturer=ACME
    emu_case "testo other manufacturer" --expect-fail --stderr -- -d $D --samples 1
    testo_options product=testo-445
    emu_case "testo other product" --expect-fail --stderr -- -d $D --samples 1
    testo_options manufacturer=
    emu_case "testo no manufacturer string" --expect-fail --stderr -- -d $D --samples 1
    testo_options product=
    emu_case "testo no product string" --expect-fail --stderr -- -d $D --samples 1
    testo_options string-stall=1
    emu_case "testo manufacturer string stalls" --expect-fail --stderr --filter "$E" -- -d $D --samples 1
    testo_options string-stall=2
    emu_case "testo product string stalls" --expect-fail --stderr --filter "$E" -- -d $D --samples 1

    # The scan's own failures: the probe returns with the device left open
    # (protocol.c:71-73, 81-82, 87-88, 93, 136 have no sr_dev_close()).
    for r in baud data flow-ctrl modem-ctrl; do
        testo_options stall=$r
        emu_case "testo scan stall $r" --expect-fail --stderr --filter "$E" -- -d $D --samples 1
    done
    testo_options in-stall=0
    emu_case "testo scan in stalls" --expect-fail --stderr -- -d $D --samples 1
    testo_options out-stall=0
    emu_case "testo scan out stalls" --expect-fail --stderr -- -d $D --samples 1
    testo_options units=7
    emu_case "testo scan unknown unit" --expect-fail --stderr -- -d $D --samples 1
    # A full packet is followed by a status-only one in the same transfer. The
    # C takes those bytes for data (protocol.c:95), so it has the 128 bytes of
    # 17 units one read before the answer is whole.
    BYTES="protocol.c:95 copies len - 2 bytes as data, the status bytes of every packet after a transfer's first"
    U17=1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1
    testo_options units=$U17 chunk=62
    emu_case "testo scan 17 units" --oracle-bug "$BYTES" --expect-fail --stderr --filter "$E" -- -d $D --samples 1
    testo_options units=$U17,1
    emu_case "testo scan 18 units" --expect-fail --stderr -- -d $D --samples 1

    # A capture's failures that end.
    for r in baud data flow-ctrl modem-ctrl; do
        testo_options stall-start=$r
        emu_case "testo start stall $r" --expect-fail --stderr --filter "$S" -- -d $D --samples 1
    done
    testo_options bad-crc=1
    emu_case "testo bad crc" -- -d $D --samples 2
    emu_case "testo bad crc log" --stderr --filter "$E; /testo: Packet has/!d;s/^.*testo: /testo: /" -- -d $D -l 5 --samples 2
    # A measurement the scan did not see: the sample counts, the value goes.
    testo_options units=1 units2=3 swap=2
    # The C's stop runs twice (api.c:267, 329), a line of the core's.
    emu_case "testo hotswap" --stderr --filter "$E; /^sr: hwdriver: testo: Device instance not active/d; s/^oxrok: warn: oxrok::cli::pipeline: //" -- -d $D --samples 2
    testo_options units=1 units2=7 swap=2
    emu_case "testo unknown unit in capture" --stderr --filter "$E; /testo: Unsupported/!d;s/^.*testo: /testo: /" -- -d $D -l 5 --samples 2
    # The same units in another order.
    I="protocol.c:281 reuses the i of the loop over the measurements (protocol.c:255), so a measurement of a later channel skips those between"
    testo_options units=1,3,5 units2=3,1,5 swap=1
    emu_case "testo reordered units" --oracle-differs "$I" -- -d $D --samples 2

    testo_options
    # Another program holds the interface. The last case left the same
    # descriptors, so the reset does not move the device from under the hold.
    if testo_hold; then
        emu_case "testo interface claimed" --expect-fail --stderr --filter "$E" -- -d $D --samples 1
        testo_release
    fi

    # An unplug. The replug binds ftdi_sio again.
    emu_case "testo unplug in a read" --unplug in=1:4 -- -d $D --samples 100
    emu_case "testo unplug at a request" --unplug out=2:3 -- -d $D --samples 100
    # After the start's last vendor request.
    emu_case "testo unplug at the start" --unplug out=0:8 --no-output -- -d $D --samples 100

    # The stop while a read is held: a limit of time, and a client's stop.
    testo_options units=1 mute=2 mute-hold=1
    emu_case "testo time limit" --timeout 5 -- -d $D --time 150
    emu_case "testo stop" --libsigrok -- -d $D --continuous --stop-after-ms 150
    # oxrok ends on the signal, from a held read; a stop it ignored would time out.
    emu_case "testo interrupt" --oracle-bug "sigrok-cli has no SIGINT handler: a signal kills it" --interrupt in=1:3 --timeout 5 -- -d $D --continuous

    # The C never ends; oxrok does, after a bound of its own.
    H="--oracle-hangs --timeout 2"
    SILENT="protocol.c:85-91 reads for ever while the chip sends its status bytes"
    NOASK="no request is sent again, and nothing ends the wait for an answer"
    PREFIX="api.c:254-255 returns on a bad prefix with no request and the bytes kept: $NOASK"
    testo_options mute=0
    emu_case "testo hang silent at scan" --oracle-bug "$SILENT" $H --expect-fail --stderr --filter "$E" -- -d $D --samples 1
    testo_options bad-prefix=0
    emu_case "testo hang bad prefix at scan" --oracle-bug "$SILENT" $H --expect-fail --stderr --filter "$E" -- -d $D --samples 1
    testo_options mute=1
    emu_case "testo hang silent in capture" --oracle-bug "api.c:285-296: $NOASK" $H --stderr --filter "$E" -- -d $D --samples 1
    testo_options bad-prefix=1
    emu_case "testo hang bad prefix in capture" --oracle-bug "$PREFIX" $H -- -d $D --samples 1
    testo_options out-stall=1
    emu_case "testo hang out stalls in capture" --oracle-bug "api.c:281-283 ignores the status of the OUT: $NOASK" $H --stderr --filter "$E" -- -d $D --samples 1
    testo_options in-stall=2
    emu_case "testo hang in stalls in capture" --oracle-bug "api.c:293-296 resubmits after any error, for ever" $H --stderr --filter "$E" -- -d $D --samples 1
    testo_options units=1 units2=$U17,1 swap=1
    emu_case "testo hang overrun in capture" --oracle-bug "api.c:237-242 returns with no request: $NOASK" $H --stderr --filter "$E" -- -d $D --samples 1
    # Two temperatures: i goes back to the first channel each time.
    testo_options units=1,1
    emu_case "testo hang same unit twice" --oracle-bug "$I, and two of a unit send the second for ever" $H -- -d $D --samples 1
    # Past 62 payload bytes: no CRC ever matches, for ever.
    testo_options units=1,3,5,24,1,3,5,24
    emu_case "testo hang status bytes in the answer" --oracle-bug "api.c:289-291 strips the status bytes once per transfer, and $BYTES" $H -- -d $D --samples 1
    testo_options
}
