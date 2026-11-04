# The IKALOGIC Scanalogic-2 (emu/src/models/ikalogic-scanalogic2.rs): a
# low speed HID device that does everything through 128 byte feature
# reports on endpoint 0. Sourced by tools/diffcli.sh --emu; see
# docs/USB.md, "Writing a device model". oxrok's port runs every case.
#
# After a status that is not "data ready" the driver waits 1.5 s of wall
# time for the next read (WAIT_DATA_READY_INTERVAL), so few cases use
# wait-trigger=, sampling= or stale-sample=.
EMU_MODEL=ikalogic-scanalogic2
EMU_DRIVER=ikalogic-scanalogic2
EMU_FULL=ikalogic-scanalogic2
EMU_GCOV_SHARED="src/usb.c"
# digest=1's runs of `@128:SHA256` packets fold into one line.
EMU_TX_FOLD=1

# The model's options for the cases that follow (see fx2lafw.sh).
sl2_options() {
    set -- options "$@"
    emu_cmd "$*" >/dev/null || emu_fail "ikalogic-scanalogic2: the emulator refused '$*'"
}

# Claims interface 0 through usbfs as another program would, until
# sl2_release (as fx2lafw_hold in fx2lafw.sh).
sl2_hold() {
    [ "$EMU_DRY" = 0 ] || return 0 # diffcli counting cases: no device, nothing to claim
    timeout 120 python3 -I -c '
import fcntl, os, signal, struct, sys
d = "/sys/bus/usb/devices/" + sys.argv[1]
bus, dev = (int(open(d + f).read()) for f in ("/busnum", "/devnum"))
fd = os.open("/dev/bus/usb/%03d/%03d" % (bus, dev), os.O_RDWR)
# usbhid until a C driver has detached it: USBDEVFS_IOCTL, USBDEVFS_DISCONNECT.
try:
    fcntl.ioctl(fd, 0xc0105512, struct.pack("iiQ", 0, 0x5516, 0))
except OSError:
    pass
fcntl.ioctl(fd, 0x8004550f, struct.pack("I", 0))  # USBDEVFS_CLAIMINTERFACE
print("held", flush=True)
signal.pause()
' "$EMU_BUSID" >"$TMP/sl2-hold" 2>&1 &
    HOLDPID=$!
    i=0
    until grep -q '^held$' "$TMP/sl2-hold" 2>/dev/null; do
        i=$((i + 1))
        if [ $i -gt 100 ] || ! kill -0 $HOLDPID 2>/dev/null; then
            emu_fail "ikalogic-scanalogic2: could not claim the interface: $(cat "$TMP/sl2-hold")"
            return 1
        fi
        sleep 0.1
    done
}

sl2_release() {
    [ "$EMU_DRY" = 0 ] || return 0
    kill $HOLDPID 2>/dev/null
    wait $HOLDPID 2>/dev/null
    HOLDPID=""
}

emu_cases() {
    S=ikalogic-scanalogic2
    # A long capture's goldens: the header, the first line of each
    # channel, and the digest of the whole output.
    B='1,/^3:/!d'
    # The trigger marker's line and those.
    T='/T/{=;b;};1,/^3:/!d'
    # oxrok's log prefix made sigrok-cli's, and the detail of the reason
    # that oxrok's message adds to sigrok-cli's own: the line stays, and a
    # one-sided rule leaves the goldens as sigrok-cli wrote them.
    O='oxrok: [a-z]+: oxrok::drivers::ikalogic_scanalogic2(::protocol)?:'
    P="s/^$O /sr: ikalogic-scanalogic2: /; s/^(Failed to open device): .*\$/\\1./; s/^(Failed to set device option '[a-z]+': invalid argument): .*\$/\\1./; s/^Failed to set channels: invalid argument: //"
    # And libsigrok's core lines, which oxrok has no counterpart of: its
    # refusal is the line after them, which stays.
    H="/^sr: hwdriver: /d; $P"
    I="sigrok-cli has no SIGINT handler"

    # The scan: RESET, INFO, the read, RESET, IDLE.
    emu_case "scan" --keep-header -- -d $S --scan
    # The driver offers no scan options: libsigrok's check refuses conn=
    # without a word before scan() runs, and nothing is found
    # (hwdriver.c:513-516, 583-586). oxrok says which option it refuses.
    emu_case "scan by conn" --oracle-differs "check_options() refuses a scan option without a word, and --scan exits 0 with nothing found, hwdriver.c:513-516, 583-586" --expect-fail --stderr --filter "$P" -- -d $S:conn=$EMU_USB --scan
    emu_case "show" --keep-header -- -d $S --show
    emu_case "get samplerate" --keep-header -- -d $S --get samplerate
    emu_case "get captureratio" --keep-header -- -d $S --get captureratio
    emu_case "set then get samplerate" --keep-header -- -d $S -c samplerate=2500000 --get samplerate
    emu_case "set then get captureratio" --keep-header -- -d $S -c captureratio=30 --get captureratio
    emu_case "set samplerate" --keep-header --no-output -- -d $S --set -c samplerate=20M
    emu_case "samplerate not listed" --expect-fail --stderr --filter "$P" -- -d $S -c samplerate=3M --samples 8
    # sr_parse_sizestring() drops the unit after a fraction
    # (strutil.c:1214 leaves s on the unit, 1244 steps over it), so under
    # sigrok-cli "2.5M" is 2, which the table refuses, and 2.5 MHz takes
    # 2500000. oxrok's parser reads 2.5M.
    emu_case "samplerate 2.5M" --oracle-bug "sr_parse_sizestring() drops the unit after a fraction, strutil.c:1214, 1244" -- -d $S -c samplerate=2.5M --samples 8 -O bits
    sl2_options serial=258 fw=1.2
    emu_case "firmware 1.2" --keep-header -- -d $S --scan
    # The serial is a u32, which the C prints with %d: over 2^31 it reads
    # negative. oxrok prints it as it is.
    sl2_options serial=3000000000
    emu_case "serial over 2^31" --oracle-differs 'g_strdup_printf("%d", dev_info.serial) on a uint32_t, api.c:110' --keep-header -- -d $S --scan
    sl2_options

    # Every samplerate code, 0x0a to 0x00. The default 1.25 kHz too.
    emu_case "default rate" -- -d $S --samples 16 -O bits
    for r in 1250 10k 50k 100k 250k 500k 1M 2500000 5M 10M 20M; do
        emu_case "rate $r" -- -d $S -c samplerate=$r --samples 16 -O bits
    done

    # Sizes around the byte (8 samples) and the packet (992 samples).
    for n in 1 7 8 9 15 16 17 991 992 993 1984 1985; do
        emu_case "samples $n" -- -d $S -c samplerate=1M --samples $n -O hex
    done
    # Digests in the log from here on.
    sl2_options digest=1
    emu_case "samples 10000" --digest --filter "$B" -- -d $S -c samplerate=1M --samples 10000 -O bits
    emu_case "samples 100000" --digest --filter "$B" -- -d $S -c samplerate=20M --samples 100000 -O bits
    # The most there is: 32765 sample bytes, 265 packets per channel, so
    # the last one is short.
    emu_case "samples 262120" --digest --filter "$B" -- -d $S -c samplerate=20M --samples 262120 -O bits
    emu_case "samples 262120 channel 3" --digest --filter "$B" -- -d $S -C 3 -c samplerate=20M --samples 262120 -O bits
    emu_case "samples 262120 ratio 50" --digest --filter "$T" -- -d $S -c samplerate=20M:captureratio=50 -t 2=r --samples 262120 -O bits
    sl2_options
    emu_case "samples over the limit" --expect-fail --stderr --filter "$P" -- -d $S --samples 262121 -O bits
    emu_case "samples 0" --expect-fail --stderr --filter "$H" -- -d $S --samples 0 -O bits
    # No limit, which the case "lib no limit" below shows the driver cannot
    # take: sigrok-cli starts no session without one (its help, no error).
    emu_case "no limit no session" --stderr --filter "$P" --no-output -- -d $S -O bits
    # --time turns into samples at the rate.
    emu_case "time 100ms at 10k" --digest --filter "$B" -- -d $S -c samplerate=10k --time 100ms -O bits

    # Output formats.
    emu_case "bits" --record -- -d $S -c samplerate=1M --samples 64 -O bits
    emu_case "ascii" -- -d $S -c samplerate=1M --samples 64 -O ascii
    emu_case "csv" -- -d $S -c samplerate=1M --samples 32 -O csv:header=false
    emu_case "vcd" -- -d $S -c samplerate=1M --samples 32 -O vcd

    # Channels: the model sends all four (no channel mask in CMD_SAMPLE),
    # the driver stops after the last enabled one and one read more. With a
    # packet still left for the read after that, it finds 05 CHANNEL ...,
    # which makes it a RESET; with none ("channels 0 1 2", one packet per
    # channel) it finds 05 63 and goes straight to IDLE.
    emu_case "channel 0" -- -d $S -C 0 -c samplerate=1M --samples 32 -O bits
    emu_case "channel 1" -- -d $S -C 1 -c samplerate=1M --samples 32 -O bits
    emu_case "channel 2" -- -d $S -C 2 -c samplerate=1M --samples 32 -O bits
    emu_case "channel 3" -- -d $S -C 3 -c samplerate=1M --samples 32 -O bits
    emu_case "channels 0 1" -- -d $S -C 0,1 -c samplerate=1M --samples 32 -O bits
    emu_case "channels 0 2" -- -d $S -C 0,2 -c samplerate=1M --samples 32 -O bits
    emu_case "channels 1 3" -- -d $S -C 1,3 -c samplerate=1M --samples 32 -O bits
    emu_case "channels 2 3" -- -d $S -C 2,3 -c samplerate=1M --samples 32 -O bits
    emu_case "channels 0 1 2" -- -d $S -C 0,1,2 -c samplerate=1M --samples 32 -O bits
    emu_case "channel 0 two packets" --digest --filter "$B" -- -d $S -C 0 -c samplerate=1M --samples 1500 -O bits
    emu_case "channels 1 2 two packets" --digest --filter "$B" -- -d $S -C 1,2 -c samplerate=1M --samples 1500 -O bits
    emu_case "channel not there" --expect-fail --stderr --filter "$P" -- -d $S -C 4 --samples 8 -O bits

    # Triggers: type and channel codes.
    emu_case "trigger 0 rising" -- -d $S -c samplerate=1M -t 0=r --samples 32 -O bits
    emu_case "trigger 1 falling" -- -d $S -c samplerate=1M -t 1=f --samples 32 -O bits
    emu_case "trigger 2 edge" -- -d $S -c samplerate=1M -t 2=e --samples 32 -O bits
    emu_case "trigger 3 rising" -- -d $S -c samplerate=1M -t 3=r --samples 32 -O bits
    emu_case "trigger 3 falling" -- -d $S -c samplerate=1M -t 3=f --samples 32 -O bits
    # All four on any edge: the channel code for all channels.
    emu_case "trigger all edges" -- -d $S -c samplerate=1M -t 0=e,1=e,2=e,3=e --samples 32 -O bits
    # Several: the last one wins (sl2_convert_trigger()).
    emu_case "trigger three edges" -- -d $S -c samplerate=1M -t 0=e,1=e,2=e --samples 32 -O bits
    emu_case "trigger mixed" -- -d $S -c samplerate=1M -t 0=r,3=f --samples 32 -O bits
    # The device has one trigger channel. oxrok takes the same one, and warns.
    emu_case "trigger mixed warns" --oracle-differs "sl2_convert_trigger() keeps the last match without a word, protocol.c:495-513" --stderr --filter "$P" -- -d $S -c samplerate=1M -t 0=r,3=f --samples 32 -O bits
    # sigrok-cli refuses a trigger on a disabled channel, so the driver's
    # own check of it (protocol.c:497) is not reached.
    emu_case "trigger on disabled channel" --expect-fail --stderr --filter "$P" -- -d $S -C 1 -c samplerate=1M -t 0=r --samples 32 -O bits
    # Two stages: "only 1 trigger stage", whose SR_ERR
    # dev_acquisition_start() drops: the capture runs without a trigger.
    # oxrok refuses it, before CMD_SAMPLE. The runner shows the C's message,
    # which an --oracle-differs case does not keep.
    emu_case "trigger two stages" --oracle-differs "dev_acquisition_start() drops sl2_convert_trigger()'s error and captures untriggered, api.c:342" --expect-fail --stderr --filter "$P" -- -d $S -c samplerate=1M -t 0=rf --samples 32 -O bits
    emu_case "trigger two stages stderr" --libsigrok --stderr -- -d $S -c samplerate=1M -t 0=rf --samples 32 -O bits
    # Types the driver does not list.
    emu_case "trigger level refused" --expect-fail --stderr --filter "$P" -- -d $S -t 0=1 --samples 8 -O bits

    # The capture ratio: pre and post samples round up to bytes.
    for c in 1 10 25 33 50 75 99 100; do
        emu_case "ratio $c" -- -d $S -c samplerate=1M:captureratio=$c -t 1=r --samples 100 -O bits
    done
    # 0.25 and 24.75 lose a sample between them: post gets it back, and
    # with it a fourth byte.
    emu_case "ratio 1 rounding" --record -- -d $S -c samplerate=1M:captureratio=1 -t 1=r --samples 25 -O bits
    # Ignored without a trigger.
    emu_case "ratio without trigger" -- -d $S -c samplerate=1M:captureratio=50 --samples 32 -O bits
    # The trigger in a later packet of the output, and a later channel.
    sl2_options digest=1
    emu_case "ratio 50 long" --digest --filter "$T" -- -d $S -c samplerate=1M:captureratio=50 -t 0=f --samples 3000 -O bits
    emu_case "ratio 50 long channels 1 3" --digest --filter "$T" -- -d $S -C 1,3 -c samplerate=1M:captureratio=50 -t 3=r --samples 3000 -O bits
    sl2_options

    # captureratio over 100 would underflow post_trigger_samples
    # (protocol.c:567), but libsigrok refuses it before config_set().
    emu_case "ratio 101 refused" --expect-fail --stderr --filter "$H" -- -d $S -c samplerate=1M:captureratio=101 -t 1=r --samples 100 -O bits

    # The status counted in reads, 1.5 s of wall time apart.
    sl2_options sampling=1
    emu_case "sampling first" -- -d $S -c samplerate=1M --samples 16 -O bits
    sl2_options wait-trigger=1 sampling=1
    emu_case "wait for trigger" -- -d $S -c samplerate=1M -t 0=r --samples 16 -O bits
    emu_case "no trigger no wait" -- -d $S -c samplerate=1M --samples 16 -O bits
    # The old buffer first: the info report of the scan, not a valid
    # status; the driver just asks again.
    sl2_options stale-sample=1
    emu_case "stale status after sample" -- -d $S -c samplerate=1M --samples 16 -O bits
    # A trigger that never comes: the driver polls for ever, under both
    # tools, and only oxrok can be interrupted. The signal comes during the
    # status wait that follows this read, which it cuts short: a status read,
    # RESET, a status read, IDLE.
    sl2_options wait-trigger=never
    emu_case "trigger never comes" --libsigrok --hangs --timeout 6 --no-output -- -d $S -c samplerate=1M -t 0=r --samples 16 -O bits
    emu_case "oxrok interrupt during a status wait" --oracle-bug "$I" --interrupt in=0:2 --no-output -- -d $S -c samplerate=1M -t 0=r --samples 16 -O bits
    # With the reset never recognized as well, the C resets for ever ("lib
    # stop never recognized" below). oxrok gives up after a hundred status
    # reads, or Ctrl-C could not end it.
    sl2_options digest=1 wait-trigger=never ignore-reset=never
    emu_case "oxrok interrupt with the reset never recognized" --oracle-bug "$I" --interrupt in=0:2 --stderr --filter "$P" -- -d $S -c samplerate=1M -t 0=r --samples 16 -O bits
    sl2_options

    # After a capture with channels left: the RESET's status.
    # The last packet again (not a status of "ready"): RESET once more.
    sl2_options stale-reset=1
    emu_case "stale status after reset" -- -d $S -C 0 -c samplerate=1M --samples 16 -O bits
    # Not a valid status at all: the driver asks again.
    sl2_options busy-reset=2
    emu_case "busy after reset" -- -d $S -C 0 -c samplerate=1M --samples 16 -O bits
    # The reset not recognized: the next packet, RESET once more.
    sl2_options ignore-reset=1
    emu_case "reset not recognized" -- -d $S -C 0 -c samplerate=1M --samples 16 -O bits
    sl2_options

    # scan(): each request of sl2_get_device_info() failing. The device
    # is then not listed, and nothing else happens.
    sl2_options stall-set=1
    emu_case "scan reset stalls" --stderr --filter "$P" -- -d $S --scan
    sl2_options stall-set=2
    emu_case "scan info stalls" --stderr --filter "$P" -- -d $S --scan
    sl2_options stall-get=1
    emu_case "scan read stalls" --stderr --filter "$P" -- -d $S --scan
    # A short answer: the C names ret, its length, as a libusb error.
    sl2_options short-get=1
    emu_case "scan read short" --oracle-differs "ret, 4, goes to libusb_error_name() as LIBUSB_TRANSFER_STALL, protocol.c:666-668" --stderr --filter "$P" -- -d $S --scan
    # The runner shows the C's message, which that case does not keep.
    emu_case "scan read short stderr" --libsigrok --stderr -- -d $S --scan
    sl2_options stall-set=3
    emu_case "scan second reset stalls" --stderr --filter "$P" -- -d $S --scan
    sl2_options stall-set=4
    emu_case "scan idle stalls" --stderr --filter "$P" -- -d $S --scan
    # The info read answers the old buffer: the driver does not look at
    # byte 0 (the wiki's "0x0A valid"), and lists version 0.0. oxrok too:
    # nothing shows what a real device does here. It warns.
    sl2_options stale-info=1
    emu_case "scan stale info" --keep-header -- -d $S --scan
    emu_case "scan stale info warns" --oracle-differs "sl2_get_device_info() does not look at byte 0, protocol.c:674-678" --stderr --filter "$P" -- -d $S --scan
    # dev_open().
    sl2_options stall-set=5
    emu_case "open reset stalls" --expect-fail --stderr --filter "$P" -- -d $S --samples 8 -O bits
    sl2_options stall-set=6
    emu_case "open idle stalls" --expect-fail --stderr --filter "$P" -- -d $S --samples 8 -O bits
    sl2_options stall-set=5
    emu_case "show with reset stalled" --expect-fail --stderr --filter "$P" -- -d $S --show
    # A capture: CMD_SAMPLE, a status read, a packet, the end.
    sl2_options stall-set=7
    emu_case "sample stalls" --stderr --filter "$P" -- -d $S --samples 8 -O bits
    sl2_options stall-get=2
    emu_case "status read stalls" --stderr --filter "$P" -- -d $S --samples 8 -O bits
    sl2_options stall-get=3
    emu_case "packet stalls" --stderr --filter "$P" -- -d $S --samples 8 -O bits
    sl2_options stall-get=6
    emu_case "last channel packet stalls" --stderr --filter "$P" -- -d $S --samples 8 -O bits
    sl2_options stall-get=8
    emu_case "ready read stalls" --stderr --filter "$P" -- -d $S --samples 8 -O bits
    sl2_options stall-set=8
    emu_case "idle after capture stalls" --stderr --filter "$P" -- -d $S --samples 8 -O bits
    sl2_options stall-set=8
    emu_case "reset after capture stalls" --stderr --filter "$P" -- -d $S -C 0 --samples 8 -O bits
    # A short packet: libusb says completed, the driver ignores
    # actual_length (sl2_receive_transfer_in), so the samples are
    # whatever the buffer held: here status bytes, exit 0. oxrok ends the
    # capture as on a failed transfer.
    A="sl2_receive_transfer_in() ignores actual_length, protocol.c:301-307, so the samples are what the buffer held"
    sl2_options short-get=3
    emu_case "short packet" --oracle-differs "$A" --stderr --filter "$P" -- -d $S -C 0 -c samplerate=1M --samples 32 -O bits
    # The second packet short: its samples repeat the first one's, and
    # nothing says so. oxrok delivers the first one's samples only.
    sl2_options short-get=4
    emu_case "short second packet" --oracle-differs "$A" --stderr --filter "$P" -- -d $S -C 0 -c samplerate=1M --samples 1984 -O hex
    # Short, with every byte used: both tools have the right samples.
    sl2_options short-get=3 short-len=6
    emu_case "short packet holding all" -- -d $S -C 0 -c samplerate=1M --samples 16 -O bits
    # A status of one byte, or none: not "data ready", under either tool; the
    # next read comes 1.5 s later.
    sl2_options sampling=1 short-get=2 short-len=1
    emu_case "status of one byte" -- -d $S -c samplerate=1M --samples 16 -O bits
    sl2_options sampling=1 short-get=2 short-len=0
    emu_case "status of no bytes" -- -d $S -c samplerate=1M --samples 16 -O bits
    # After a capture the C takes its old buffer, a packet, for a valid status
    # and resets; oxrok asks again.
    sl2_options short-get=5 short-len=0
    emu_case "status of no bytes after a capture" --oracle-differs "sl2_receive_transfer_in() ignores actual_length and reads its old buffer as the status, protocol.c:323-334" -- -d $S -C 0 -c samplerate=1M --samples 8 -O bits
    sl2_options

    # Another program holds the interface.
    if sl2_hold; then
        emu_case "interface claimed by another program" --stderr --filter "$P" -- -d $S --scan
        sl2_release
    fi

    # Unplugged: during the scan, the open, the capture. A case's model
    # counts the transfers from 1: 1 RESET, 2 INFO, in 1 the read, 3
    # RESET, 4 IDLE, 5 RESET, 6 IDLE, 7 CMD_SAMPLE, in 2 the status.
    emu_case "unplug scan reset" --stderr --filter "$P" --unplug out=0:1 -- -d $S --scan
    emu_case "unplug scan info" --stderr --filter "$P" --unplug out=0:2 -- -d $S --scan
    emu_case "unplug scan read" --stderr --filter "$P" --unplug in=0:1 -- -d $S --scan
    emu_case "unplug scan second reset" --stderr --filter "$P" --unplug out=0:3 -- -d $S --scan
    # Not unplugged after the scan's last request: the next step is no
    # transfer (the release, or a new look at the bus), so what it finds
    # is a race with the detach.
    emu_case "unplug open reset" --expect-fail --stderr --filter "$P" --unplug out=0:5 -- -d $S --samples 8 -O bits
    emu_case "unplug open idle" --stderr --filter "$P" --unplug out=0:6 -- -d $S --samples 8 -O bits
    emu_case "unplug after sample" --stderr --filter "$P" --unplug out=0:7 -- -d $S --samples 8 -O bits
    emu_case "unplug at status" --stderr --filter "$P" --unplug in=0:2 -- -d $S --samples 8 -O bits
    emu_case "unplug at first packet" --stderr --filter "$P" --unplug in=0:3 -- -d $S --samples 8 -O bits
    emu_case "unplug at last packet" --stderr --filter "$P" --unplug in=0:7 -- -d $S --samples 8 -O bits
    emu_case "unplug at the channel after" --stderr --filter "$P" --unplug in=0:4 -- -d $S -C 0 --samples 8 -O bits
    emu_case "unplug before the reset" --stderr --filter "$P" --unplug in=0:5 -- -d $S -C 0 --samples 8 -O bits
    emu_case "unplug at the status before idle" --stderr --filter "$P" --unplug in=0:6 -- -d $S -C 0 --samples 8 -O bits
    emu_case "unplug before idle" --stderr --filter "$P" --unplug out=0:8 -- -d $S -C 0 --samples 8 -O bits
    # Between two status reads, the driver's own poll fails to submit.
    sl2_options sampling=1
    emu_case "unplug between status reads" --stderr --filter "$P" --unplug in=0:2 -- -d $S --samples 8 -O bits
    sl2_options

    # libsigrok-run: what sigrok-cli cannot do. No limit: 0 bytes asked,
    # and sample_packet %= num_sample_packets divides by 0 (protocol.c:317).
    # Library clients only, and only because the model answers a capture
    # of 0 bytes with 05 60, its own choice (the model's doc): a device that
    # never says data ready would make the driver poll for ever.
    # --expect-fail takes any failure: only the .tx is pinned, not status 136.
    emu_case "lib no limit" --libsigrok --expect-fail --no-output -- -d $S -O bits
    # A stop, as a client's button (sigrok-cli stops only on a key with
    # --continuous, which the driver does not offer). libsigrok-run's stop
    # runs at high priority, ahead of the next USB completion.
    emu_case "lib stop at header" --libsigrok --no-output -- -d $S --samples 800 --stop-after 0 -O bits
    sl2_options stale-sample=1
    emu_case "lib stop at header stale status" --libsigrok --no-output -- -d $S --samples 800 --stop-after 0 -O bits
    sl2_options
    # Mid-capture, the stop comes from the datafeed callback: libusb can reap
    # the next sample packets in the pass that delivered this one, ahead of
    # any callback of the main loop (tools/libsigrok-run.c), and the .tx would
    # hold 1 or 2 packets more, about once in 30 runs under load. The driver's
    # stop only sets a flag (api.c:403-408), so it can run there.
    emu_case "lib stop during packets" --libsigrok -- -d $S -C 0 --samples 5000 --stop-after 100 --stop-in-callback -O bits
    # The reset of a stop never recognized: for ever.
    sl2_options wait-trigger=never ignore-reset=never
    emu_case "lib stop never recognized" --libsigrok --hangs --timeout 3 --no-output -- -d $S -t 0=r --samples 800 --stop-after 0 -O bits
    sl2_options
}
