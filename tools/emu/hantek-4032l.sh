# The Hantek 4032L logic analyzer (emu/src/models/hantek-4032l.rs): a
# vendor USB device, 04b5:4032, with no firmware or bitstream to
# upload. Sourced by tools/diffcli.sh --emu; see docs/USB.md, "Writing
# a device model".
EMU_MODEL=hantek-4032l
EMU_DRIVER=hantek-4032l
# oxrok has the driver: its cases must run full.
EMU_FULL=hantek-4032l
EMU_GCOV_SHARED="src/usb.c"

# The model's options for the cases that follow (see fx2lafw.sh).
h4032l_options() {
    set -- options "$@"
    emu_cmd "$*" >/dev/null || emu_fail "hantek-4032l: the emulator refused '$*'"
}

# Claims interface 0 through usbfs as another program would, until
# h4032l_release (as hantek_hold in hantek-dso.sh).
h4032l_hold() {
    [ "$EMU_DRY" = 0 ] || return 0 # diffcli counting cases: no device, nothing to claim
    timeout 120 python3 -I -c '
import fcntl, os, signal, struct, sys
d = "/sys/bus/usb/devices/" + sys.argv[1]
bus, dev = (int(open(d + f).read()) for f in ("/busnum", "/devnum"))
fd = os.open("/dev/bus/usb/%03d/%03d" % (bus, dev), os.O_RDWR)
fcntl.ioctl(fd, 0x8004550f, struct.pack("I", 0))  # USBDEVFS_CLAIMINTERFACE
print("held", flush=True)
signal.pause()
' "$EMU_BUSID" >"$TMP/h4032l-hold" 2>&1 &
    HOLDPID=$!
    i=0
    until grep -q '^held$' "$TMP/h4032l-hold"; do
        i=$((i + 1))
        if [ $i -gt 100 ] || ! kill -0 $HOLDPID 2>/dev/null; then
            emu_fail "hantek-4032l: could not claim the interface: $(cat "$TMP/h4032l-hold")"
            return 1
        fi
        sleep 0.1
    done
}

h4032l_release() {
    kill $HOLDPID 2>/dev/null
    wait $HOLDPID 2>/dev/null
    HOLDPID=""
}

emu_cases() {
    H=hantek-4032l
    # oxrok's log prefix as sigrok-cli's: only the prefix goes, the text
    # stays compared.
    N='s/^oxrok: [a-z]+: oxrok::drivers::hantek_4032l[a-z0-9_:]*: /sr: hantek-4032l: /'
    # The driver's own messages on stderr, without sigrok-cli's prefix
    # (and the timestamps of -l 5). What else the tools print is dropped:
    # that it fails is the exit status's to say.
    E="$N; /hantek-4032l: /!d; s/.*hantek-4032l: //"
    # The last stderr line, sigrok-cli's own.
    L='$!d'
    # oxrok's wording of a refused option as sigrok-cli's: the detail after
    # "invalid argument". Which option was refused, and that it was, stays
    # compared.
    W='s/(invalid argument).*$/\1./'
    # The address in --show.
    C='s/conn: [0-9]+\.[0-9]+/conn: BUS.ADDR/'
    # A few channels of each group, and what a capture of them is held
    # to: the first 64 samples, the trigger marker and, through
    # --digest, the SHA-256 of the whole output.
    CH=A0,B0,A1,B1,A2,A7,B15
    B='1,8b; /^T:/b; d'
    # The driver takes 2048 samples and up (sigrok-cli checks --samples
    # against that too).
    S=2048

    h4032l_options
    emu_case "scan" --keep-header -- -d $H --scan
    emu_case "by address" --digest --filter "$B" -- -d $H:conn=$EMU_USB -C $CH --samples $S -O bits
    emu_case "by ids" --digest --filter "$B" -- -d $H:conn=04b5.4032 -C $CH --samples $S -O bits
    emu_case "bad conn" --stderr --filter 's/^oxrok: [a-z]+: oxrok::drivers::hantek_4032l: /sr: usb: /' -- -d $H:conn=nonsense --scan
    emu_case "show" --keep-header --filter "$C" -- -d $H --show
    emu_case "show A" --keep-header --filter "$C" -- -d $H -g A --show
    emu_case "show B" --keep-header --filter "$C" -- -d $H -g B --show
    emu_case "default capture" --record --digest --filter "$B" -- -d $H -C $CH --samples $S -O bits
    emu_case "all channels" --digest --filter '1,40!d' -- -d $H --samples $S -O hex
    emu_case "2048 samples 100M" --digest --filter '1~16!d' -- -d $H -C A0,B5 -c samplerate=100M --samples $S -O bits
    # sigrok-cli prints 3000; the driver asks for 3072 and reads them.
    emu_case "rounded up 3000" --digest --filter '1~16!d' -- -d $H -C A3 --samples 3000 -O bits
    emu_case "400M 256k samples" --digest --filter '1~64!d' -- -d $H -C B15 -c samplerate=400M --samples 256k -O bits
    emu_case "1k 64k samples" --digest --filter '1~64!d' -- -d $H -C A8 -c samplerate=1k --samples 64k -O bits
    # Every rate of the list: each code of samplerates_hw[].
    for r in 1k 2k 4k 8k 16k 31250 62500 125k 250k 500k 625k 781250 1M 1250k 1562500 2M 2500k 3125k 4M 5M 6250k 10M 12500k 20M 25M 40M 50M 80M 160M 200M 320M 400M; do
        emu_case "samplerate $r" --digest --filter '1,3!d' -- -d $H -C A0 -c samplerate=$r --samples $S -O bits
    done
    for kv in captureratio=0 captureratio=50 captureratio=99 captureratio=100; do
        emu_case "set $kv" --digest --filter "$B" -- -d $H -C $CH -c $kv --samples $S -O bits
    done
    # Thresholds: the PWM values in the configure.
    emu_case "threshold A 1.5" --digest --filter '1,3!d' -- -d $H -g A -c voltage_threshold=1.5-1.5 -C A0 --samples $S -O bits
    emu_case "threshold B -6 6" --digest --filter '1,3!d' -- -d $H -g B -c voltage_threshold=-6-6 -C B0 --samples $S -O bits
    emu_case "threshold B 6" --digest --filter '1,3!d' -- -d $H -g B -c voltage_threshold=6-6 -C B0 --samples $S -O bits
    emu_case "threshold A -6" --digest --filter '1,3!d' -- -d $H -g A -c voltage_threshold=-6--6 -C B0 --samples $S -O bits
    # Out of the listed range, which the driver takes all the same: the
    # reference is clamped at -5 V (PWM 0).
    emu_case "threshold A 9" --digest --filter '1,3!d' -- -d $H -g A -c voltage_threshold=9-9 -C A0 --samples $S -O bits
    # The other way, the reference is clamped at 10 V, PWM 4096, one more
    # than the 4095 of protocol.c's comment; sent all the same.
    emu_case "threshold A minus 9" --digest --filter '1,3!d' -- -d $H -g A -c voltage_threshold=-9--9 -C A0 --samples $S -O bits
    emu_case "external clock" --digest --filter "$B" -- -d $H -c external_clock=1 -C $CH --samples $S -O bits
    for src in ACLK BCLK; do
        for edge in rising falling both; do
            emu_case "external clock $src $edge" --digest --filter '1,3!d' -- -d $H -c external_clock=1:external_clock_source=$src:clock_edge=$edge -C A0 --samples $S -O bits
        done
    done
    # A source and an edge without the external clock: the rate code.
    emu_case "clock edge without external clock" --digest --filter '1,3!d' -- -d $H -c clock_edge=falling:external_clock_source=BCLK -C A0 --samples $S -O bits
    # Triggers on the counter the model plays: A0 is bit 0, B0 bit 1,
    # A1 bit 2, ... (channel n is A(n/2) or B(n/2)).
    emu_case "trigger A1 rising" --record --digest --filter "$B" -- -d $H -C $CH -t A1=r --samples $S -O bits
    emu_case "trigger B0 falling" --digest --filter "$B" -- -d $H -C $CH -t B0=f --samples $S -O bits
    emu_case "trigger A2 edge" --digest --filter "$B" -- -d $H -C $CH -t A2=e --samples $S -O bits
    emu_case "trigger levels" --digest --filter "$B" -- -d $H -C $CH -t A0=1,B0=0,A1=1 --samples $S -O bits
    emu_case "trigger level and edge" --digest --filter "$B" -- -d $H -C $CH,B7 -t B7=1,A2=r -c captureratio=10 --samples $S -O bits
    emu_case "trigger B15 rising" --digest --filter '1~16b; /^T:/b; d' -- -d $H -C A0,B15 -t B15=r -c captureratio=50 --samples $S -O bits
    # A ratio of 100: pre_trigger_size equals the sample count, the
    # trigger point falls just past the capture, and sigrok-cli shows
    # no trigger marker.
    emu_case "trigger captureratio=100" --digest --filter "$B" -- -d $H -C $CH -t A1=r -c captureratio=100 --samples $S -O bits
    # A trigger at sample 0: no samples before the trigger marker.
    emu_case "trigger ratio 0" --digest --filter "$B" -- -d $H -C $CH -t A1=f -c captureratio=0 --samples $S -O bits
    # The trigger past the first transfer (127 samples), in a data
    # transfer.
    emu_case "trigger ratio 30" --digest --filter '1,3b; /^T:/b; d' -- -d $H -C A0,A3 -t A3=r -c captureratio=30 --samples $S -O bits
    # Two stages: sigrok-cli reaches api.c:529-530, oxrok's Trigger::check
    # refuses before the driver. Only the wording of that one message is
    # made the same; that it is refused stays compared, by the exit status,
    # and that nothing is sent, by the transactions.
    emu_case "two trigger stages" --expect-fail --stderr --filter "$N; s/.*Only (one trigger stage|1 trigger stages are) supported.*/Only one trigger stage supported./; /^Only one/!d" -- -d $H -t A0=01 --samples $S
    # A second edge is refused, whichever kind it is.
    for t in A0=r,B0=r A0=r,B0=f A0=f,B0=e; do
        emu_case "two edges $t" --expect-fail --stderr --filter "$E" -- -d $H -t $t --samples $S
    done
    # The gets: what the driver keeps.
    for k in samplerate captureratio limit_samples external_clock external_clock_source clock_edge; do
        emu_case "get $k" -- -d $H --get $k
    done
    emu_case "get conn" --filter "s/^'[0-9]+\.[0-9]+'\$/'BUS.ADDR'/" -- -d $H --get conn
    emu_case "get A threshold" -- -d $H -g A --get voltage_threshold
    emu_case "get B threshold set" -- -d $H -g B -c voltage_threshold=3.3-3.3 --get voltage_threshold
    emu_case "get limit_samples set" -- -d $H -c limit_samples=5000 --get limit_samples
    emu_case "get samplerate set" -- -d $H -c samplerate=625k --get samplerate
    emu_case "get external clock set" -- -d $H -c external_clock=1:external_clock_source=BCLK:clock_edge=both --get clock_edge
    # The threshold belongs to a channel group.
    emu_case "get threshold without group" --expect-fail --stderr --filter "$L" -- -d $H --get voltage_threshold
    # api.c:414-415 masks the rounded count with 0xfffffe00, a 32 bit
    # constant, so a count of 4 Gi samples and more loses its upper
    # bits: 2^32 + 2048 is taken as 2048 (a libsigrok bug, exit 0).
    # oxrok keeps the count, which is past 64M, and refuses it; the
    # .oracle golden holds the 2048 sigrok-cli prints.
    emu_case "get limit_samples 4G" --oracle-differs "api.c:414-415: num_samples &= 0xfffffe00, a 32 bit mask on a 64 bit count" --expect-fail --stderr --filter "$N; /^[0-9]+\$|hantek-4032l: /!d; s/.*hantek-4032l: //" -- -d $H -c limit_samples=4294969344 --get limit_samples
    # Values the driver refuses.
    for kv in samplerate=3M limit_samples=1000 limit_samples=70M; do
        emu_case "set $kv" --expect-fail --stderr --filter "$E" -- -d $H -c $kv --samples $S
    done
    # Names not in the lists: std_str_idx() fails, with no message of
    # the driver's own.
    for kv in external_clock_source=CCLK clock_edge=up; do
        emu_case "set $kv" --expect-fail --stderr --filter "$L; $W" -- -d $H -c $kv --samples $S
    done
    # sigrok-cli's and the generic checks, before the driver.
    emu_case "samples below the minimum" --expect-fail --stderr --filter "$L" -- -d $H --samples 100
    emu_case "samples above the maximum" --expect-fail --stderr --filter "$L" -- -d $H --samples 70M
    emu_case "continuous refused" --expect-fail --stderr --filter "$L" -- -d $H --continuous
    emu_case "get samplerate in a group" --expect-fail --stderr --filter "$L; $W" -- -d $H -g A --get samplerate
    emu_case "set captureratio above 100" --expect-fail --stderr --filter "$L; $W" -- -d $H -c captureratio=150 --samples $S
    emu_case "time limit" --digest --filter '1,3!d' -- -d $H -C A0 -c samplerate=1M --time 10 -O bits
    # The FPGA version in the driver's debug output.
    emu_case "debug fpga version" --stderr --filter "$N; /FPGA version/!d; s/.*hantek-4032l: //" -- -l 5 -d $H --show

    # FPGA version 0: no external clock, one data transfer at a time.
    h4032l_options fpga=0
    emu_case "fpga 0 show" --keep-header --filter "$C" -- -d $H --show
    emu_case "fpga 0 capture" --digest --filter '1~16!d' -- -d $H -C A0,B3 --samples 8192 -O bits
    emu_case "fpga 0 trigger" --digest --filter "$B" -- -d $H -C $CH -t B1=r -c captureratio=20 --samples $S -O bits
    emu_case "fpga 0 external clock" --expect-fail --stderr --filter "$L; $W" -- -d $H -c external_clock=1 --samples $S
    # More status polls before the capture is complete, and none.
    h4032l_options fill=5
    emu_case "fill 5" --digest --filter "$B" -- -d $H -C $CH --samples $S -O bits
    h4032l_options fill=0
    emu_case "fill 0" --digest --filter "$B" -- -d $H -C $CH --samples $S -O bits

    # Another program holds the interface: the claim fails.
    h4032l_options
    if h4032l_hold; then
        emu_case "interface claimed by another program" --expect-fail --stderr --filter "$E" -- -d $H --samples $S
        h4032l_release
    fi

    # Faults. The open's status command and its answer refused.
    for f in out@1 in@1; do
        h4032l_options stall=$f
        emu_case "fault open stall $f" --expect-fail --stderr --filter "$E" -- -d $H --samples $S
    done
    # Leftovers before every answer: the open reads past them (up to
    # ten reads), the capture drops them.
    h4032l_options stale=3
    emu_case "fault stale 3" --digest --filter "$B" -- -d $H -C $CH --samples $S -O bits
    h4032l_options stale=9
    emu_case "fault stale 9" --digest --filter "$B" -- -d $H -C $CH --samples $S -O bits
    h4032l_options stale=10
    emu_case "fault stale 10" --expect-fail --stderr --filter "$E" -- -d $H --samples $S
    # The reset request refused at the start.
    h4032l_options stall=reset
    emu_case "fault reset stall" --expect-fail --stderr --filter "$E" -- -d $H --samples $S
    # A garbled end word: the driver says so and ends all the same.
    h4032l_options end=0x12345678
    emu_case "fault end word" --stderr --filter "$E" -- -d $H --samples $S
    # IN transfers refused during the capture (the open reads #1; the
    # capture's status reads are #2 to #5, the stream's start #6, the
    # data from #7; a start word refused is read again). The driver logs a failed transfer at debug level
    # only and goes on: a status read refused counts as an answer (its
    # buffer holds the previous one), a data transfer refused is
    # submitted again.
    for f in in@2 in@3 in@4 in@5 in@6 in@7 in@8; do
        h4032l_options stall=$f
        emu_case "fault capture stall $f" --digest --filter "$B" -- -d $H -C $CH --samples 8192 -O bits
    done
    # A status command refused (OUT #3; #1 is the open's, #2 the
    # configure): the two reads that follow find nothing and time out
    # (500 ms each), then the driver asks again.
    h4032l_options stall=out@3
    emu_case "fault status command stall" --digest --filter "$B" -- -d $H -C $CH --samples $S -O bits

    # What the C driver never ends and oxrok does.
    IGNORED="protocol.c:229-230 logs a failed transfer at debug level and goes on, with no timeout on the capture"
    # The configure refused: the C driver does not notice, and the
    # board, never configured, answers status 0 for ever.
    h4032l_options stall=out@2
    emu_case "fault configure stall" --oxrok-fails "endpoint stalled" --oracle-bug "$IGNORED" --oracle-hangs --timeout 2 --stderr --filter "$N" -- -d $H --samples $S
    # The get refused (OUT #5): the board goes on answering the status,
    # which the driver drops while it waits for the start word, for
    # ever.
    h4032l_options stall=out@5
    emu_case "fault get stall" --oxrok-fails "endpoint stalled" --oracle-bug "$IGNORED" --oracle-hangs --timeout 2 --stderr --filter "$N" -- -d $H --samples $S
    # A trigger the counter never meets (A0 high and B0 rising: B0 rises
    # where A0 is low): status 1 for ever. This runs libsigrok alone, through
    # the runner, as sigrok-cli has no way to stop it. oxrok waits the same
    # way until stopped; its unit test status_answers_of_1_wait_until_a_stop
    # shows that, as the model answers status polls at once and so cannot
    # pace a signal among status 1 answers.
    h4032l_options
    emu_case "trigger never fires hangs" --libsigrok --hangs --timeout 2 --stderr --no-output -- -d $H -t A0=1,B0=r --samples $S
    # Ctrl-C ends oxrok while it polls, here before any status answer: the
    # status command refused makes the two reads after it time out (500 ms
    # each), so that the signal lands on a known number of transfers: no
    # read completes.
    h4032l_options stall=out@3
    emu_case "interrupt while polling" --interrupt out=2:3 --oracle-bug "sigrok-cli has no SIGINT handler" --no-output -- -d $H -t A0=1,B0=r --samples $S

    # A slow board: the stream holds after its first data transfer (IN #7),
    # the oldest held transfer times out with one packet, the
    # stream goes on, both tools take the data ("partial then release", also
    # with one transfer at a time).
    h4032l_options hold=1 partial=512 release=1
    emu_case "partial then release" --digest --filter "$B" -- -d $H -C $CH --samples $S -O bits
    h4032l_options hold=1 partial=512 release=1 fpga=0
    emu_case "partial then release fpga 0" --digest --filter "$B" -- -d $H -C $CH --samples $S -O bits
    # Ctrl-C during the data, the device holding what follows IN #7, so that
    # the signal lands on a known number of transfers: oxrok ends with what
    # came. With a partial, what the cancel gives counts too.
    h4032l_options hold=1
    emu_case "interrupt while held" --interrupt in=6:7 --oracle-bug "sigrok-cli has no SIGINT handler" --digest --filter '1,3!d' -- -d $H -C A0 --samples $S -O bits
    h4032l_options hold=1 partial=512
    emu_case "interrupt while held with a partial" --interrupt in=6:7 --oracle-bug "sigrok-cli has no SIGINT handler" --digest --filter '1,3!d' -- -d $H -C A0 --samples $S -O bits
    # dev_acquisition_stop() in libsigrok, on the samples that came before
    # the hold (127 and 512): the transfers held come back to the abort.
    h4032l_options hold=1
    emu_case "lib stop while held" --libsigrok --digest --filter '1,3!d' -- -d $H -C A0 --samples $S -O bits --stop-after 639

    h4032l_options
    # Unplugged while the driver polls the status (after IN #3), and in
    # the middle of the data (after IN #9 of a 64k sample capture).
    emu_case "unplug while polling" --oxrok-fails "device disconnected" --stderr --filter "$E" --unplug in=6:3 -- -d $H --samples $S
    # In the data the C driver logs one line per transfer in flight (32
    # here, resubmit_transfer() failing for each) and oxrok, whose stream
    # reports the device gone once, one. The filter keeps the first of the C's
    # and words it as oxrok's; only those repeats and that wording go.
    D="$E; /^resubmit_transfer: /{x; /seen/{x;d}; s/.*/seen/; x; s/^resubmit_transfer: (LIBUSB_ERROR_NO_DEVICE)\$/Data transfer failed: \1./}"
    emu_case "unplug mid data" --oxrok-fails "device disconnected" --stderr --filter "$D" --unplug in=6:9 -- -d $H -C A0 --samples 64k -O bits
    h4032l_options
}
