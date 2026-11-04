# The Kecheng KC-330B sound level meter (emu/src/models/kecheng-kc-330b.rs):
# commands on bulk OUT 2, answers on bulk IN 1. Sourced by tools/diffcli.sh
# --emu; see docs/USB.md, "Writing a device model". oxrok has no port of the
# driver yet: every case is an oracle case.
#
# The live path polls on wall time, 125 ms apart (api.c:42, the only interval
# a client can have: see "set sample_interval"), so captures are short.
#
# The clock: the C has kecheng_kc_330b_set_date_time() (protocol.c:232) and
# nothing calls it (api.c:141 is a TODO), so no C case can reach it and no
# golden holds a wall time.
EMU_MODEL=kecheng-kc-330b
EMU_DRIVER=kecheng-kc-330b
EMU_GCOV_SHARED="src/usb.c"

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

    # The scan: identify, on 5 ms and 10 ms timeouts.
    emu_case "scan" --keep-header -- -d $S --scan
    # NO_OPTS (api.c:296): libsigrok's check refuses conn= before scan().
    emu_case "scan by conn" --no-output -- -d $S:conn=$EMU_USB --scan
    # api.c:215 answers a tuple for a key whose type is uint64 (hwdriver.c:113);
    # show.c's g_variant_get_uint64() is a GLib critical, which ends sigrok-cli.
    emu_case "show" --expect-fail --stderr -- -d $S --show
    for k in limit_samples sample_interval spl_weight_freq spl_weight_time data_source; do
        emu_case "get $k" --keep-header -- -d $S --get $k
    done
    # api.c:219: there is no way to tell, so the get fails.
    emu_case "get datalog" --keep-header --expect-fail --stderr -- -d $S --get datalog
    emu_case "get continuous" --keep-header --expect-fail --stderr -- -d $S --get continuous
    emu_case "get samples" --keep-header -- -d $S -c limit_samples=7 --get limit_samples
    emu_case "set then get weight freq" --keep-header -- -d $S -c spl_weight_freq=C --get spl_weight_freq
    emu_case "set then get weight time" --keep-header -- -d $S -c spl_weight_time=S --get spl_weight_time
    emu_case "set then get source" --keep-header -- -d $S -c data_source=Memory --get data_source
    # dev_close() sends the configuration only when it is not dirty
    # (api.c:190), against its comment: these --set runs send nothing.
    emu_case "set weight freq" --no-output -- -d $S --set -c spl_weight_freq=C
    emu_case "set weight time" --no-output -- -d $S --set -c spl_weight_time=S
    emu_case "set source" --no-output -- -d $S --set -c data_source=Memory
    emu_case "weight freq refused" --expect-fail --stderr -- -d $S -c spl_weight_freq=Z --samples 1
    emu_case "weight time refused" --expect-fail --stderr -- -d $S -c spl_weight_time=X --samples 1
    emu_case "source refused" --expect-fail --stderr -- -d $S -c data_source=Tape --samples 1
    # api.c:259: std_u64_tuple_idx() takes a tuple, and sr_config_set() has
    # checked for a uint64: a GLib critical for any value.
    emu_case "set sample_interval" --expect-fail --stderr -- -d $S -c sample_interval=125 --samples 1
    emu_case "time refused" --expect-fail --stderr -- -d $S --time 100ms
    emu_case "frames refused" --expect-fail --stderr -- -d $S --frames 1
    emu_case "samples refused" --expect-fail --stderr -- -d $S --samples x

    # The live path: configure, status, then 08 at each interval.
    emu_case "samples 1" -- -d $S --samples 1
    emu_case "samples 2" -- -d $S --samples 2
    emu_case "samples 5" -- -d $S --samples 5
    emu_case "A F" -- -d $S -c spl_weight_freq=A:spl_weight_time=F --samples 2
    emu_case "A S" -- -d $S -c spl_weight_freq=A:spl_weight_time=S --samples 2
    emu_case "C F" -- -d $S -c spl_weight_freq=C:spl_weight_time=F --samples 2
    emu_case "C S" -- -d $S -c spl_weight_freq=C:spl_weight_time=S --samples 2
    emu_case "output analog" -- -d $S --samples 2 -O analog
    emu_case "output csv" -- -d $S --samples 2 -O csv:header=false
    # hwdriver.c refuses a limit of 0.
    emu_case "samples 0 refused" --expect-fail --stderr -- -d $S --samples 0
    # The stops a sigrok-cli without a key cannot reach. Live never ends by
    # itself, and a stop at the first sample lands in the 125 ms pause.
    emu_case "lib stop after 1" --libsigrok -- -d $S -O analog --stop-after 1
    emu_case "lib stop after 2" --libsigrok -- -d $S -O analog --stop-after 2
    emu_case "lib stop between requests" --libsigrok -- -d $S -O analog --stop-after-ms 60
    emu_case "lib continuous runs on" --libsigrok --hangs --timeout 2 --filter '2,$d' -- -d $S --continuous -O analog

    # An inactive meter reads 30.0 and is logged at, and the capture goes on
    # (api.c:344-350).
    kc_options inactive=1
    emu_case "inactive" -- -d $S --samples 2
    emu_case "inactive message" --stderr -- -d $S --samples 1
    kc_options

    # The identify: the answer names the model.
    kc_options model=Sound_meter
    emu_case "model string" --keep-header -- -d $S --scan
    kc_options model=123456789012345678901234567890
    emu_case "model 30 bytes" --keep-header -- -d $S --scan
    kc_options model=1234567890123456789012345678901
    emu_case "model 31 bytes" --keep-header --no-output -- -d $S --scan
    kc_options id-length=0
    emu_case "model length 0" --keep-header -- -d $S --scan
    kc_options id-length=31
    emu_case "model length 31" --keep-header --no-output -- -d $S --scan
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
        emu_case "status $f" --expect-fail --stderr -- -d $S --samples 1
    done

    # The first request failing refuses the start (api.c:405), and a later one
    # ends the capture (protocol.c:68).
    kc_options stall-out=08@1
    emu_case "first request fails" --expect-fail --stderr -- -d $S --samples 2
    kc_options stall-out=08@2
    emu_case "second request fails" -- -d $S --samples 3

    # An answer that fails or does not parse leaves the state on LIVE_SPL_WAIT
    # and nothing submits another transfer (protocol.c:149, protocol.c:153):
    # the capture never ends.
    for f in stall-in=08@2 silent=08@2 garbage=08@2 short=08@2; do
        kc_options $f
        emu_case "live answer $f" --hangs --timeout 2 -- -d $S --samples 3
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
    emu_case "unplug before the first request" --unplug in=1:3 --expect-fail --stderr -- -d $S --samples 3

    # Pulled out with the second request's read on its way: the callback sees
    # LIBUSB_TRANSFER_NO_DEVICE (protocol.c:137) and stops.
    emu_case "unplug during a read" --unplug out=2:5 -- -d $S --samples 3

    # The log path. A meter with an empty log: header and end (api.c:357), with
    # nothing configured, as the setting of data_source left it dirty.
    kc_options stored=0
    emu_case "log empty" --no-output -- -d $M --samples 5
    emu_case "log empty no limit" --no-output --libsigrok -- -d $M -O analog
    kc_options

    # A log with samples. api.c:367 sends the interval as a tuple, while
    # sigrok-cli's session.c:365 takes a uint64: a GLib critical, which ends it
    # before the first request.
    emu_case "log" --expect-fail --stderr -- -d $M --samples 5
    emu_case "log no limit" --expect-fail --stderr -- -d $M --continuous
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
    # The log info fails: the start is refused and the stop still configures.
    for f in stall-out=05 stall-in=05 silent=05 garbage=05 short=05; do
        kc_options $f
        emu_case "log info $f" --expect-fail --stderr -- -d $M --samples 5
    done
    kc_options log-interval=7
    emu_case "log interval 7" --expect-fail --stderr -- -d $M --samples 5
    kc_options

    # The stop's configure fails: it logs, and that is all.
    kc_options stored=0 stall-out=01
    emu_case "log empty, no configure on the wire" --no-output -- -d $M --samples 5
    kc_options
}
