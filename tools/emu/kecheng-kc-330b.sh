# The Kecheng KC-330B sound level meter (emu/src/models/kecheng-kc-330b.rs):
# commands on bulk OUT 2, answers on bulk IN 1. Sourced by tools/diffcli.sh
# --emu; see docs/USB.md, "Writing a device model". The port is
# src/drivers/kecheng_kc_330b.
#
# The live path polls on wall time, 125 ms apart (api.c:42, the only interval
# a client can have: see "set sample_interval"), so captures are short.
#
# The clock: the C has kecheng_kc_330b_set_date_time() (protocol.c:232) and
# nothing calls it (api.c:141 is a TODO), so no case reaches it and no golden
# holds a wall time. oxrok has no counterpart.
#
# This model needs a quiet host. The oracle's timeouts are 5, 10, 15 and 40 ms
# and cannot move: under load sigrok-cli's scan misses its identify (api.c:75)
# and finds nothing, and most cases then fail on the oracle's side. oxrok's own
# are 100 to 250 ms.
EMU_MODEL=kecheng-kc-330b
EMU_DRIVER=kecheng-kc-330b
EMU_GCOV_SHARED="src/usb.c"
# oxrok has the driver: its cases must run full.
EMU_FULL=kecheng-kc-330b

# The model's options for the cases that follow; none restores them.
kc_options() {
    set -- options "$@"
    emu_cmd "$*" >/dev/null || emu_fail "kecheng-kc-330b: the emulator refused '$*'"
}

# Claims interface 0 through usbfs as another program would, until kc_release
# (as fx2lafw_hold in fx2lafw.sh).
kc_hold() {
    [ "$EMU_DRY" = 0 ] || return 0 # diffcli counting cases: no device, nothing to claim
    timeout 120 python3 -I -c '
import fcntl, os, signal, struct, sys
d = "/sys/bus/usb/devices/" + sys.argv[1]
bus, dev = (int(open(d + f).read()) for f in ("/busnum", "/devnum"))
fd = os.open("/dev/bus/usb/%03d/%03d" % (bus, dev), os.O_RDWR)
fcntl.ioctl(fd, 0x8004550f, struct.pack("I", 0))  # USBDEVFS_CLAIMINTERFACE
print("held", flush=True)
signal.pause()
' "$EMU_BUSID" >"$TMP/kc-hold" 2>&1 &
    HOLDPID=$!
    i=0
    until grep -q '^held$' "$TMP/kc-hold" 2>/dev/null; do
        i=$((i + 1))
        if [ $i -gt 100 ] || ! kill -0 $HOLDPID 2>/dev/null; then
            emu_fail "kecheng-kc-330b: could not claim the interface: $(cat "$TMP/kc-hold")"
            return 1
        fi
        sleep 0.1
    done
}

kc_release() {
    [ "$EMU_DRY" = 0 ] || return 0
    kill $HOLDPID 2>/dev/null
    wait $HOLDPID 2>/dev/null
    HOLDPID=""
}

emu_cases() {
    S=kecheng-kc-330b
    M="$S -c data_source=Memory"
    # The driver's own lines at -l 4, no other module's, without the time.
    D='s/^sr: \[[^]]*\] //;/^kecheng-kc-330b: /!d'
    # GLib's critical on api.c:367's tuple is what "log" below stops at.
    G=--survive-glib-critical
    # oxrok's log lines as sigrok-cli's.
    N='s/^oxrok: [a-z]+: oxrok::drivers::kecheng_kc_330b[a-z_:]*: /sr: kecheng-kc-330b: /'
    # A refused start, however each tool puts it: sigrok-cli's two lines, of
    # which the first names the session, oxrok's session's one with the
    # driver's error. The driver logged its cause itself, at debug in both.
    R="$N
/^sr: session: /d
s/^(Failed to start session\.|device error: .*)\$/start refused/"
    # dev_close() configures when the settings are not dirty (api.c:190): the
    # last two lines of the C's --get. oxrok's close sends a configuration only
    # when dirty, so no --show or --get clears a log (api.c:430). This drops
    # only that pair at the very end, and from the tool that sends it; a case
    # without it ("set ...", "get sample_interval") shows the difference.
    T='$!N; ${/^bulk_out 02 01[0-9a-f]{12} -> 7\nbulk_in 81 [0-9]+ -> 81$/d}; P; D'

    CLOSE_BUG="dev_close() sends the configuration only when it is not dirty (api.c:190, against its comment at api.c:187): a set never reaches the meter"
    TUPLE_GET="config_get() answers a tuple for a uint64 key (api.c:215, hwdriver.c:113)"
    TUPLE_SHOW="$TUPLE_GET: show.c's g_variant_get_uint64() is a GLib critical, which ends sigrok-cli"
    TUPLE_SET="config_set() reads a tuple from a uint64 key (api.c:259, hwdriver.c:113): a GLib critical, which ends sigrok-cli"
    TUPLE_META="the log's meta packet holds a tuple for a uint64 key (api.c:367): session.c:365's g_variant_get_uint64() is a GLib critical, which ends sigrok-cli"
    NO_CONN="NO_OPTS (api.c:296): libsigrok's check refuses conn=, so nothing is found"
    WAITS="an answer that fails or does not parse leaves the state on LIVE_SPL_WAIT and nothing submits another transfer (protocol.c:149, protocol.c:153), so the capture never ends"
    NO_INT="sigrok-cli has no SIGINT handler"
    # What the session's stop of a refused log start sends (session.c:831,
    # api.c:429), against api.c:433, on a log it never read. oxrok sends no
    # configure: sigrok-cli exits 1 there, so --oracle-differs cannot run, and
    # this drops the pair from both tools. So oxrok's lack of it is pinned by the
    # unit test a_refused_log_start_sends_no_configure, and the C's by the
    # library case "lib log info fails".

    # The scan: identify, on 5 ms and 10 ms timeouts.
    emu_case "scan" --keep-header -- -d $S --scan
    emu_case "scan by conn" --oracle-differs "$NO_CONN" --keep-header -- -d $S:conn=$EMU_USB --scan
    emu_case "scan by another conn" --keep-header --no-output -- -d $S:conn=99.99 --scan
    emu_case "show" --oracle-bug "$TUPLE_SHOW" --keep-header -- -d $S --show
    for k in limit_samples spl_weight_freq spl_weight_time data_source; do
        emu_case "get $k" --keep-header --tx-filter "$T" -- -d $S --get $k
    done
    emu_case "get sample_interval" --oracle-differs "$TUPLE_GET" --keep-header --tx-filter "$T" -- -d $S --get sample_interval
    # api.c:219: there is no way to tell, so the get fails.
    emu_case "get datalog" --keep-header --expect-fail --stderr --filter "$R" -- -d $S --get datalog
    emu_case "get continuous" --keep-header --expect-fail --stderr --filter "$R" -- -d $S --get continuous
    emu_case "get samples" --keep-header --tx-filter "$T" -- -d $S -c limit_samples=7 --get limit_samples
    emu_case "set then get weight freq" --oracle-differs "$CLOSE_BUG" --keep-header -- -d $S -c spl_weight_freq=C --get spl_weight_freq
    emu_case "set then get weight time" --oracle-differs "$CLOSE_BUG" --keep-header -- -d $S -c spl_weight_time=S --get spl_weight_time
    # In memory mode oxrok's close holds it back too (api.c:430): no difference.
    emu_case "set then get source" --keep-header -- -d $S -c data_source=Memory --get data_source
    # These --set runs send nothing from the C; oxrok's goes out.
    emu_case "set weight freq" --oracle-differs "$CLOSE_BUG" --no-output -- -d $S --set -c spl_weight_freq=C
    emu_case "set weight time" --oracle-differs "$CLOSE_BUG" --no-output -- -d $S --set -c spl_weight_time=S
    # A value it has already is no change: nothing goes out from either, as by
    # api.c:430 it would clear the log.
    emu_case "set default source" --no-output -- -d $S --set -c data_source=Live
    emu_case "set default weights" --no-output -- -d $S --set -c spl_weight_freq=A:spl_weight_time=F
    emu_case "set source" --no-output -- -d $S --set -c data_source=Memory
    emu_case "weight freq refused" --expect-fail --stderr --filter "$R" -- -d $S -c spl_weight_freq=Z --samples 1
    emu_case "weight time refused" --expect-fail --stderr --filter "$R" -- -d $S -c spl_weight_time=X --samples 1
    emu_case "source refused" --expect-fail --stderr --filter "$R" -- -d $S -c data_source=Tape --samples 1
    # oxrok takes the interval in ms, the key's unit. So the other indices,
    # which no C client can reach, go out here.
    emu_case "set sample_interval" --oracle-bug "$TUPLE_SET" -- -d $S -c sample_interval=125 --samples 1
    for i in 500 1000 2000 5000 10000 60000; do
        emu_case "sample_interval $i" --oracle-bug "$TUPLE_SET" -- -d $S -c sample_interval=$i --samples 1
    done
    emu_case "sample_interval 500 paced" --oracle-bug "$TUPLE_SET" -- -d $S -c sample_interval=500 --samples 3
    emu_case "sample_interval refused" --oracle-bug "$TUPLE_SET" --expect-fail --stderr --filter "$R" -- -d $S -c sample_interval=126 --samples 1
    emu_case "time refused" --expect-fail --stderr --filter "$R" -- -d $S --time 100ms
    emu_case "frames refused" --expect-fail --stderr --filter "$R" -- -d $S --frames 1
    emu_case "samples refused" --expect-fail --stderr --filter "$R" -- -d $S --samples x

    # The live path: configure, status, then 08 at each interval.
    emu_case "samples 1" -- -d $S --samples 1
    emu_case "samples 2" --record -- -d $S --samples 2
    emu_case "samples 5" -- -d $S --samples 5
    emu_case "A F" -- -d $S -c spl_weight_freq=A:spl_weight_time=F --samples 2
    emu_case "A S" -- -d $S -c spl_weight_freq=A:spl_weight_time=S --samples 2
    emu_case "C F" -- -d $S -c spl_weight_freq=C:spl_weight_time=F --samples 2
    emu_case "C S" -- -d $S -c spl_weight_freq=C:spl_weight_time=S --samples 2
    emu_case "output analog" -- -d $S --samples 2 -O analog
    emu_case "output csv" -- -d $S --samples 2 -O csv:header=false
    # hwdriver.c refuses a limit of 0.
    emu_case "samples 0 refused" --expect-fail --stderr --filter "$R" -- -d $S --samples 0
    # The stops a sigrok-cli without a key cannot reach. Live never ends by
    # itself, and a stop at the first sample lands in the 125 ms pause.
    emu_case "lib stop after 1" --libsigrok -- -d $S -O analog --stop-after 1
    emu_case "lib stop after 2" --libsigrok -- -d $S -O analog --stop-after 2
    emu_case "lib stop between requests" --libsigrok -- -d $S -O analog --stop-after-ms 60
    emu_case "lib continuous runs on" --libsigrok --hangs --timeout 2 --filter '2,$d' -- -d $S --continuous -O analog
    # oxrok's stop, on the same pause: the signal lands at the second sample.
    emu_case "interrupt between samples" --oracle-bug "$NO_INT" --interrupt in=1:5 -- -d $S --continuous

    # An inactive meter reads 30.0 and is logged at, and the capture goes on
    # (api.c:344-350).
    kc_options inactive=1
    emu_case "inactive" -- -d $S --samples 2
    emu_case "inactive message" --stderr --filter "$R" -- -d $S --samples 1
    kc_options

    # The identify: the answer names the model.
    kc_options model=Sound_meter
    emu_case "model string" --keep-header -- -d $S --scan
    kc_options model=123456789012345678901234567890
    emu_case "model 30 bytes" --keep-header -- -d $S --scan
    kc_options model=1234567890123456789012345678901
    # 33 bytes for the C's 32 byte read, a babble; nusb's read is a packet. Both
    # refuse it, so this hides the host's note, not a transfer.
    emu_case "model 31 bytes" --keep-header --no-output --tx-filter 's/ \(babble: .*\)$//' -- -d $S --scan
    kc_options id-length=0
    emu_case "model length 0" --keep-header -- -d $S --scan
    kc_options id-length=31
    emu_case "model length 31" --keep-header --no-output -- -d $S --scan
    # A length past the answer: the C names bytes of its stack (api.c:97-98),
    # masked here; oxrok only what came, in protocol.rs's unit test.
    kc_options id-length=30
    emu_case "model length past the answer" --keep-header --filter 's/^(kecheng-kc-330b - Kecheng ).* with 1 channel/\1MODEL with 1 channel/' -- -d $S --scan
    for f in stall-out=02 stall-in=02 silent=02 garbage=02; do
        kc_options $f
        emu_case "identify $f" --keep-header --no-output -- -d $S --scan
    done
    # One byte of a two byte minimum.
    kc_options model= short=02
    emu_case "identify short" --keep-header --no-output -- -d $S --scan
    kc_options

    # A configure that fails at the start is ignored (api.c:340), and the one
    # dev_close() sends is too.
    for f in stall-out=01 stall-in=01 silent=01 garbage=01 short=01; do
        kc_options $f
        emu_case "configure $f" -- -d $S --samples 1
    done
    kc_options

    # A status that fails refuses the start (api.c:342).
    for f in stall-out=04 stall-in=04 silent=04 garbage=04 short=04; do
        kc_options $f
        emu_case "status $f" --expect-fail --stderr --filter "$R" -- -d $S --samples 1
    done

    # The first request failing refuses the start (api.c:405), and a later one
    # ends the capture (protocol.c:68).
    kc_options stall-out=08@1
    emu_case "first request fails" --expect-fail --stderr --filter "$R" -- -d $S --samples 2
    kc_options stall-out=08@2
    emu_case "second request fails" -- -d $S --samples 3

    # An answer that fails or does not parse. oxrok asks again at the next
    # interval and gets its samples.
    for f in stall-in=08@2 silent=08@2 garbage=08@2 short=08@2; do
        kc_options $f
        emu_case "live answer $f" --oracle-bug "$WAITS" --oracle-hangs --timeout 6 -- -d $S --samples 3
    done
    # Every answer bad: oxrok gives up after MAX_BAD_ANSWERS.
    for f in stall-in=08 silent=08 garbage=08 short=08; do
        kc_options $f
        emu_case "live answers $f" --oracle-bug "$WAITS" --oracle-hangs --timeout 6 --no-output -- -d $S --samples 3
    done
    kc_options

    # Another program holds the interface: the scan's identify cannot get it
    # (usbfs claims on the first URB, api.c:75 has no claim of its own), so no
    # device is found and dev_open()'s failures stay out of reach.
    kc_hold
    emu_case "interface held" --no-output -- -d $S --scan
    kc_release

    # Pulled out: in the pause after a sample, the next request fails
    # (protocol.c:68), and the close's configure fails too. The fourth IN is
    # the first sample's; the third is the status's, and the start fails.
    emu_case "unplug between samples" --unplug in=1:4 -- -d $S --samples 3
    emu_case "unplug before the first request" --unplug in=1:3 --expect-fail --stderr --filter "$R" -- -d $S --samples 3
    # Pulled out with the second request's read on its way: the callback sees
    # LIBUSB_TRANSFER_NO_DEVICE (protocol.c:137) and stops.
    emu_case "unplug during a read" --unplug out=2:5 -- -d $S --samples 3

    # The log path. A meter with an empty log: header and end (api.c:357), with
    # nothing configured, as the setting of data_source left it dirty.
    kc_options stored=0
    emu_case "log empty" --no-output -- -d $M --samples 5
    emu_case "log empty no limit" --no-output --libsigrok -- -d $M -O analog
    kc_options

    # A log with samples. sigrok-cli dies on the meta packet before the first
    # request; oxrok reads them all, and sends no configure: the log stays.
    emu_case "log" --oracle-bug "$TUPLE_META" -- -d $M --samples 5
    emu_case "log no limit" --oracle-bug "$TUPLE_META" --digest --filter '1,3!d' -- -d $M --continuous
    for n in 1 62 63 200; do
        emu_case "log limit $n" --oracle-bug "$TUPLE_META" --digest --filter '1,3!d' -- -d $M --samples $n
    done
    for n in 1 62 63 64 126 127; do
        kc_options stored=$n
        emu_case "log stored $n" --oracle-bug "$TUPLE_META" --digest --filter '1,3!d' -- -d $M --continuous
    done
    # The samples carry the log's weightings, not what was asked for
    # (api.c:354). Asking for them is a setting beyond the data source: read to
    # its end, the log then goes with the configure that applies them.
    kc_options stored=2 log-time=0 log-freq=0
    emu_case "log A F" --oracle-bug "$TUPLE_META" -- -d $M:spl_weight_freq=C:spl_weight_time=S --continuous
    kc_options stored=2 log-time=1 log-freq=0
    emu_case "log A S" --oracle-bug "$TUPLE_META" -- -d $M --continuous
    kc_options stored=2 log-time=0 log-freq=1
    emu_case "log C F" --oracle-bug "$TUPLE_META" -- -d $M --continuous
    kc_options stored=2 log-time=1 log-freq=1
    emu_case "log C S" --oracle-bug "$TUPLE_META" -- -d $M --continuous
    # No --record below: the harness records no --oracle-bug case, so the log's
    # replays are the unit tests in src/drivers/kecheng_kc_330b/mod.rs.
    kc_options stored=32000 digest=1
    # 508 chunks: the requests stay in the .tx, every offset and count, and the
    # answers are in the digest of the output.
    emu_case "log full" --oracle-bug "$TUPLE_META" --digest --filter '1,3!d' --tx-filter '/^bulk_in 81 128 -> @127:/d' -- -d $M --continuous
    # A new interval, the same, in the only unit oxrok has.
    kc_options stored=2
    emu_case "log then a new interval" --oracle-bug "$TUPLE_SET" -- -d $M:sample_interval=1000 --continuous
    # A chunk that fails: what came is kept, and the log is not cleared.
    for f in stall-out=07@2 stall-in=07@2 silent=07@2 garbage=07@2 short=07@2; do
        kc_options stored=100 $f
        emu_case "log chunk $f" --oracle-bug "$TUPLE_META" --digest --filter '1,3!d' -- -d $M --continuous
    done
    kc_options

    # The runner's analog output module stops at the same tuple, in analog.c,
    # so it needs $G to go on. Then api.c:405: buf_len is 4, the check is
    # len != 1, so the start fails on a request that went out. The chunk stays
    # unread, and the stop's configure (api.c:429) gets it in a one byte read,
    # a babble (see the model). So no client reads the log.
    emu_case "lib log" --libsigrok --expect-fail --stderr --filter "$D" -- -d $M -l 4 $G
    emu_case "lib log limit 5" --libsigrok --expect-fail --stderr --filter "$D" -- -d $M -l 4 $G --samples 5
    emu_case "lib log limit 200" --libsigrok --expect-fail --stderr --filter "$D" -- -d $M -l 4 $G --samples 200
    for n in 1 62 63 64; do
        kc_options stored=$n
        emu_case "lib log stored $n" --libsigrok --expect-fail --stderr --filter "$D" -- -d $M -l 4 $G
    done
    kc_options stored=32000
    emu_case "lib log full" --libsigrok --expect-fail --stderr --filter "$D" -- -d $M -l 4 $G
    # The request itself refused.
    kc_options stall-out=07
    emu_case "lib log request fails" --libsigrok --expect-fail --stderr --filter "$D" -- -d $M -l 4 $G
    kc_options stall-out=05
    emu_case "lib log info fails" --libsigrok --expect-fail --stderr --filter "$D" -- -d $M -l 4
    # The log info fails: the start is refused, and only the C configures.
    for f in stall-out=05 stall-in=05 silent=05 garbage=05 short=05; do
        kc_options $f
        emu_case "log info $f" --expect-fail --stderr --filter "$R" --tx-filter "$T" -- -d $M --samples 5
    done
    kc_options log-interval=7
    emu_case "log interval 7" --expect-fail --stderr --filter "$R" --tx-filter "$T" -- -d $M --samples 5
    kc_options

    # The stop's configure fails: it logs, and that is all.
    kc_options stored=0 stall-out=01
    emu_case "log empty, no configure on the wire" --no-output -- -d $M --samples 5
    kc_options
}
