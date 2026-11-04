# The Lascar EL-USB data loggers (emu/src/models/lascar-el-usb.rs).
# Sourced by tools/diffcli.sh --emu; see docs/USB.md, "Writing a device
# model".
#
# What the cases do not reach in the C driver, and why:
# - api.c:100-101, the claim failing in dev_open(): the scan's bulk reads
#   fail first when another program holds the interface ("held interface"),
#   and nothing stages a claim between the scan and the open.
# - api.c:114 (dev_close() of a closed device), 136, 142 (no sdi), 150-151,
#   176-177 (a key the driver lacks: libsigrok's check_key() refuses it first),
#   250-251 (a profile the scan rejects never opens), protocol.c:341 (the same
#   profile test), 491-493 (the same again).
# - api.c:302, protocol.c:105, 221, 366, 481: allocation failures.
# - protocol.c:64-66: libusb_open() failing; the node is world accessible.
# - api.c:330-332, protocol.c:126, 245-246, 561-565: a submit failing right
#   after one that worked, which takes an unplug timed between two calls of
#   the host; an unplug armed on a transfer lands on the next read's
#   completion or the submit before it, not these.
# - protocol.c:374: same as 64-66, in the scan.
EMU_MODEL=lascar-el-usb
EMU_DRIVER=lascar-el-usb
EMU_GCOV_SHARED="src/usb.c"
EMU_TX_FOLD=1
# oxrok has the driver: its cases must run full.
EMU_FULL=lascar-el-usb

# The model's options for the cases that follow (see fx2lafw.sh).
el_options() {
    set -- options "$@"
    emu_cmd "$*" >/dev/null || emu_fail "lascar-el-usb: the emulator refused '$*'"
}

# Claims interface 0 through usbfs as another program would, until
# el_release (as fx2lafw_hold in fx2lafw.sh).
el_hold() {
    [ "$EMU_DRY" = 0 ] || return 0 # diffcli counting cases: no device, nothing to claim
    timeout 120 python3 -I -c '
import fcntl, os, signal, struct, sys
d = "/sys/bus/usb/devices/" + sys.argv[1]
bus, dev = (int(open(d + f).read()) for f in ("/busnum", "/devnum"))
fd = os.open("/dev/bus/usb/%03d/%03d" % (bus, dev), os.O_RDWR)
fcntl.ioctl(fd, 0x8004550f, struct.pack("I", 0))  # USBDEVFS_CLAIMINTERFACE
print("held", flush=True)
signal.pause()
' "$EMU_BUSID" >"$TMP/el-hold" 2>&1 &
    HOLDPID=$!
    i=0
    until grep -q '^held$' "$TMP/el-hold" 2>/dev/null; do
        i=$((i + 1))
        if [ $i -gt 100 ] || ! kill -0 $HOLDPID 2>/dev/null; then
            emu_fail "lascar-el-usb: could not claim the interface: $(cat "$TMP/el-hold")"
            return 1
        fi
        sleep 0.1
    done
}

el_release() {
    [ "$EMU_DRY" = 0 ] || return 0
    kill $HOLDPID 2>/dev/null
    wait $HOLDPID 2>/dev/null
    HOLDPID=""
}

emu_cases() {
    L=lascar-el-usb
    C=$L:conn=10c4.0002
    A="s/[0-9]+[.][0-9]+( [(]current[)])/BUS.ADDR\\1/"
    G="s/'[0-9]+[.][0-9]+'/'BUS.ADDR'/"
    # The driver's and sigrok-cli's own lines at -l 4 and below: hwdriver.c's
    # lines under the driver's name go (no driver logs them), and session.c's
    # with its time stamp, whose failure "Failed to start session." repeats.
    # oxrok's reason for a refusal becomes sr_strerror()'s words; the driver's
    # own line says what went wrong.
    D='(^| |::)lascar[-_]el[-_]usb[a-z_:]*: '
    E="/$D|^(Failed|Device|device |No |This|Unable|Error)/!d; /: (Opening device instance|Starting acquisition|Stopping acquisition)[.]/d; s/^.*$D//; s/^(Failed to get 'datalog'): .*\$/\\1: generic\\/unspecified error/; s/^(Failed to set device option 'datalog'): .*\$/\\1: generic\\/unspecified error./; s/^(Failed to open device): .*\$/\\1./; s/^device error: (Failed to open device).*\$/\\1./; s/^(device error: .*|device disconnected)\$/Failed to start session./"
    # A long log: the first lines, and the digest of all of it.
    H="1,10!d"

    # Every profile of the C table (protocol.c:33-56), and either side.
    for el_id in 0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 255; do
        el_options model=$el_id
        case $el_id in
            3 | 9 | 11 | 12 | 13 | 16) emu_case "scan model $el_id" --keep-header -- -d $C --scan ;;
            0) emu_case "scan model 0" --no-output -- -d $C -l 4 --scan ;;
            *) emu_case "scan model $el_id" --stderr --filter "$E" -- -d $C -l 4 --scan ;;
        esac
    done
    el_options

    # Selection.
    emu_case "scan" --keep-header -- -d $C --scan
    emu_case "scan by address" --keep-header -- -d $L:conn=$EMU_USB --scan
    emu_case "scan without conn" --no-output -- -d $L --scan
    emu_case "scan other ids" --no-output -- -d $L:conn=10c4.0003 --scan
    emu_case "scan bad conn" --no-output -- -d $L:conn=nonsense --scan

    # Options.
    emu_case "show" --keep-header --filter "$A" -- -d $C --show
    emu_case "get conn" --filter "$G" -- -d $C --get conn
    emu_case "get limit_samples" -- -d $C --get limit_samples
    emu_case "set and get limit_samples" -- -d $C -c limit_samples=7 --get limit_samples
    emu_case "set limit_samples" --no-output -- -d $C --set -c limit_samples=7
    emu_case "get datalog off" -- -d $C --get datalog
    emu_case "get samplerate" --expect-fail --stderr --filter "$E" -- -d $C --get samplerate
    el_options logging=1
    emu_case "get datalog on" -- -d $C --get datalog
    emu_case "show logging" --keep-header --filter "$A" -- -d $C --show
    emu_case "set datalog off" --no-output -- -d $C --set -c datalog=off
    el_options logging=1 cfg-silent=2
    emu_case "set datalog off cfg-silent" --expect-fail --stderr --filter "$E" -- -d $C -l 4 --set -c datalog=off
    el_options
    emu_case "set datalog on" --record --no-output -- -d $C --set -c datalog=on
    emu_case "set datalog off when off" --no-output -- -d $C --set -c datalog=off
    # protocol.c:591 clears the top three bits of byte 0x21 and keeps the rest.
    el_options flag-bits=fe
    emu_case "set datalog on with other flags" --no-output -- -d $C --set -c datalog=on
    el_options model=9
    emu_case "set datalog on model 9" --no-output -- -d $C --set -c datalog=on
    el_options

    # Captures, each profile.
    emu_case "capture" --record -- -d $C --samples 100
    for el_id in 9 11 12 13 16; do
        el_options model=$el_id
        emu_case "capture model $el_id" -- -d $C --samples 100
    done
    el_options model=13 unit=1
    emu_case "fahrenheit model 13" -- -d $C --samples 100
    el_options unit=1
    emu_case "fahrenheit" -- -d $C --samples 100
    # protocol.c:235 takes any other as Celsius.
    el_options unit=7
    emu_case "unit 7" -- -d $C --samples 100
    el_options

    # Channels.
    emu_case "temp only" -- -d $C -C Temp --samples 100
    emu_case "hum only" -- -d $C -C Hum --samples 100
    el_options model=9
    emu_case "co channel" --record -- -d $C -C CO --samples 100
    el_options

    # Outputs.
    emu_case "csv" --filter "/^;/d" -- -d $C --samples 100 -O csv
    el_options model=9
    emu_case "co csv" --filter "/^;/d" -- -d $C --samples 100 -O csv
    el_options

    # Limits: a limit the C stores and does not use (api.c:174, no reader);
    # oxrok cuts the output and reads the memory all the same.
    N="api.c:174 stores the limit and nothing reads it, so it gives all the samples"
    emu_case "limit 5" --oracle-differs "$N" -- -d $C --samples 5
    emu_case "limit 20" -- -d $C --samples 20
    el_options model=9
    emu_case "limit 5 co" --oracle-differs "$N" -- -d $C --samples 5
    el_options
    emu_case "time refused" --expect-fail --stderr --filter "$E" -- -d $C --time 100
    emu_case "frames refused" --expect-fail --stderr --filter "$E" -- -d $C --frames 1
    emu_case "continuous refused" --expect-fail --stderr --filter "$E" -- -d $C --continuous

    # Log contents.
    el_options logged=0
    emu_case "empty log" --no-output -- -d $C --samples 10
    el_options logged=1
    emu_case "one sample" -- -d $C --samples 10
    el_options logged=255
    emu_case "first block less one" --digest --filter "$H" -- -d $C --samples 1000
    el_options logged=257
    emu_case "over the first block" --digest --filter "$H" -- -d $C --samples 1000
    el_options logged=16382
    emu_case "full log" --digest --filter "$H" -- -d $C --samples 16382
    # More samples than the memory holds: the header's bytes end it.
    el_options logged=20000 memory=1024
    emu_case "count beyond the memory" --digest --filter "$H" -- -d $C --samples 1000
    el_options memory=1024
    emu_case "small memory" -- -d $C --samples 1000
    # The header says less than the count.
    el_options logged=300 log-size=512
    emu_case "header smaller than the count" --digest --filter "$H" -- -d $C --samples 1000
    el_options block=4096
    emu_case "blocks fill the read" -- -d $C --samples 100
    el_options logged=2000 block=64 memory=8192
    emu_case "64 byte blocks" --digest --filter "$H" -- -d $C --samples 2000
    # Half steps: "temp[j] = buf[i * 2] / 2 - 40" and "rh[j] = buf[i * 2 + 1] /
    # 2" (protocol.c:426, 430) divide integers, under the comment "Celsius is
    # stored in half-degree increments" and with one digit declared; eltuxusb
    # keeps the halves (el_parse.py:65, 147).
    N="protocol.c:426,430 divide integers, losing the half degrees and percents"
    el_options pattern=half
    emu_case "half steps" --oracle-differs "$N" -- -d $C --samples 100
    emu_case "half steps csv" --oracle-differs "$N" --filter "/^;/d" -- -d $C --samples 100 -O csv
    el_options pattern=half unit=1
    emu_case "half steps fahrenheit" --oracle-differs "$N" -- -d $C --samples 100
    # protocol.c:432: 0 degrees and 0 % is skipped.
    el_options invalid=3
    emu_case "invalid samples" -- -d $C --samples 100
    el_options invalid=1
    emu_case "all invalid" --no-output -- -d $C --samples 100
    el_options invalid=3 unit=1
    emu_case "invalid fahrenheit" -- -d $C --samples 100
    # Calibration.
    el_options model=9 co-high=0.25 co-low=-1000
    emu_case "co clamps negatives" -- -d $C --samples 100
    el_options model=9 co-high=2.5 co-low=7.5
    emu_case "co other calibration" -- -d $C --samples 100
    # (s * high + low) / 1e6 at digits 0: big enough to show in analog's text.
    el_options model=9 co-high=2000000 co-low=-1000
    emu_case "co large calibration" -- -d $C --samples 100
    el_options model=9 logged=65000 memory=2048
    emu_case "co count beyond the memory" --digest --filter "$H" -- -d $C --samples 2000
    el_options

    # Short or queued answers the C takes.
    el_options stale=300
    emu_case "stale data flushed" --keep-header -- -d $C --scan
    emu_case "stale data capture" -- -d $C --samples 100
    el_options
    for el_c in 0/ffff 2/2 2/1; do
        el_options stall-ctl=$el_c
        emu_case "mode request $el_c stalled" -- -d $C --samples 100
    done
    el_options stall-ctl=0/ffff stall-ctl=2/2 stall-ctl=2/1
    emu_case "all mode requests stalled" -- -d $C --samples 100
    el_options

    # Configuration answers at the scan.
    for el_f in cfg-silent cfg-garbage cfg-long cfg-short-header cfg-short cfg-nobody stall-out stall-in; do
        el_options $el_f=1
        emu_case "scan $el_f" --stderr --filter "$E" -- -d $C -l 4 --scan
    done
    el_options stall-in=2
    emu_case "scan stall-in body" --stderr --filter "$E" -- -d $C -l 4 --scan
    el_options config-size=0
    emu_case "scan empty block" --no-output -- -d $C -l 4 --scan
    el_options

    # Configuration answers at the capture and the options.
    for el_f in cfg-silent cfg-garbage cfg-long cfg-short-header cfg-short cfg-nobody; do
        el_options $el_f=2
        emu_case "capture $el_f" --expect-fail --stderr --filter "$E" -- -d $C -l 4 --samples 10
        emu_case "get datalog $el_f" --expect-fail --stderr --filter "$E" -- -d $C -l 4 --get datalog
        emu_case "set datalog $el_f" --expect-fail --stderr --filter "$E" -- -d $C -l 4 --set -c datalog=on
    done
    el_options

    # Queued past the configuration block: api.c:312 reads 256 bytes into
    # resp[4].
    el_options tail=2
    emu_case "tail 2" -- -d $C --samples 100
    el_options tail=64
    emu_case "tail 64" -- -d $C --samples 100
    # "resp[4]", then libusb_bulk_transfer(usb->devhdl, LASCAR_EP_IN, resp,
    # 256, &ret, 5) (api.c:272, 312): 256 bytes run over the stack frame and
    # sigrok-cli dies of a segmentation fault.
    el_options tail=256
    emu_case "tail 256" --oracle-bug "api.c:312 reads 256 bytes into resp[4]" -- -d $C --samples 100
    el_options

    # The log's answers. Answer 5 is the log header: 2 for the scan's
    # config, 2 for lascar_proc_config()'s.
    el_options stall-in=5
    emu_case "log request stalled" --expect-fail --stderr --filter "$E" -- -d $C -l 4 --samples 10
    el_options stall-out=3
    emu_case "log command stalled" --expect-fail --stderr --filter "$E" -- -d $C -l 4 --samples 10
    el_options log-garbage=1
    emu_case "log header garbage" --expect-fail --stderr --filter "$E" -- -d $C -l 4 --samples 10
    el_options log-short-header=1
    emu_case "log header short" --expect-fail --stderr --filter "$E" -- -d $C -l 4 --samples 10
    # A one-shot STALL of a block: protocol.c:546 skips it and goes on.
    el_options stall-in=7
    emu_case "block stalled once" -- -d $C --samples 100
    el_options stall-in=6
    emu_case "first block stalled once" -- -d $C --samples 100
    # Held to the driver's 100 ms and given then (protocol.c:539).
    el_options hold=1
    emu_case "first block timed out with data" -- -d $C --samples 100
    el_options hold-empty=1
    emu_case "first block timed out empty" -- -d $C --samples 100
    el_options hold-empty=3
    emu_case "later block timed out empty" -- -d $C --samples 100
    # No data at all: the first read times out and the header's 0 ends it.
    el_options memory=0
    emu_case "no memory" --no-output -- -d $C --samples 100
    el_options

    # Saves.
    el_options stall-out=3
    emu_case "save command stalled" --expect-fail --stderr --filter "$E" -- -d $C -l 4 --set -c datalog=on
    el_options stall-out=4
    emu_case "save block stalled" --expect-fail --stderr --filter "$E" -- -d $C -l 4 --set -c datalog=on
    el_options save-answer=00
    emu_case "save wrong answer" --expect-fail --stderr --filter "$E" -- -d $C -l 4 --set -c datalog=on
    el_options save-silent=1
    emu_case "save no answer" --expect-fail --stderr --filter "$E" -- -d $C -l 4 --set -c datalog=on
    el_options

    # protocol.c:558-566 resubmits for ever, on timeouts and on a halted
    # endpoint. oxrok gives up after 30 reads: 3 s of timeouts.
    N="protocol.c:558-566 resubmits for ever"
    el_options log-stop=0
    emu_case "device stops sending" --oracle-bug "$N" --oracle-hangs --timeout 8 --stderr --filter "$E" -- -d $C --samples 10
    el_options halt-in=6
    emu_case "endpoint halted" --oracle-bug "$N" --oracle-hangs --timeout 4 --stderr --filter "$E" -- -d $C --samples 10
    # sigrok-cli has no SIGINT handler. Held, so that the signal lands first.
    el_options log-stop=0
    emu_case "interrupted download" --oracle-bug "sigrok-cli has no SIGINT handler" --interrupt in=2:5 --no-output -- -d $C --samples 10
    el_options

    # Unplugs.
    emu_case "unplug at the scan" --unplug out=2:1 --no-output -- -d $C --scan
    emu_case "unplug mid log" --unplug in=2:8 -- -d $C --samples 100
    emu_case "unplug at the log command" --unplug out=2:3 --expect-fail --no-output -- -d $C --samples 100
    emu_case "unplug at the log header" --unplug in=2:5 --expect-fail --no-output -- -d $C --samples 100
    emu_case "unplug at the save" --unplug out=2:3 --expect-fail --stderr --filter "$E" -- -d $C -l 4 --set -c datalog=on

    emu_case "unplug at the config header" --unplug in=2:1 --no-output -- -d $C --scan
    # Which line says so depends on how soon the detach shows in the device list.
    emu_case "unplug after the scan" --unplug in=2:2 --expect-fail --no-output -- -d $C --samples 100
    emu_case "unplug before the log mode setup" --unplug out=0:6 --expect-fail --no-output -- -d $C --samples 100
    emu_case "unplug before the save" --unplug in=2:4 --expect-fail --stderr --filter "$E" -- -d $C -l 4 --set -c datalog=on

    # Another program holds the interface: the scan's bulk reads fail
    # (protocol.c:108, 116), so it finds nothing and never gets to the
    # claim of dev_open().
    if el_hold; then
        emu_case "held interface" --no-output -- -d $C --scan
        el_release
    fi

    # libsigrok-run: a stop from the client, and no limit.
    emu_case "lib no limit" --libsigrok -- -d $C -O analog
    emu_case "lib stop at header" --libsigrok --no-output -- -d $C --samples 10 --stop-after 0 -O analog
    el_options logged=2000
    emu_case "lib stop during log" --libsigrok -- -d $C --samples 2000 --stop-after 100 --stop-in-callback -O analog
    el_options
}
