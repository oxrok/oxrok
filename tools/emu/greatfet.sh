# Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
# The GreatFET One (emu/src/models/greatfet.rs): firmware facts from
# greatscottgadgets/greatfet and libgreat, the driver's from libsigrok.
# Sourced by tools/diffcli.sh --emu; see docs/USB.md, "Writing a device
# model".
#
# One transfer is 262144 bytes: that many samples with 5 to 8 channels,
# or 2, 4 or 8 times as many with 3 or 4, 2 and 1 channels. Each capture
# case's sample limit falls in the last transfer `in-limit` lets out (the
# model holds what follows, so the count is fixed).
EMU_MODEL=greatfet
EMU_DRIVER=greatfet
EMU_GCOV_SHARED="src/usb.c"
# oxrok has the driver: its cases must run full.
EMU_FULL=greatfet

# The model's options for the cases that follow.
gf_options() {
    set -- options "$@"
    emu_cmd "$*" >/dev/null || emu_fail "greatfet: the emulator refused '$*'"
}

# Claims interface 1 through usbfs, as another program would, until
# gf_release (at most 120 s). A claim is no transfer, so the model sees
# nothing of it.
gf_hold() {
    [ "$EMU_DRY" = 0 ] || return 0 # diffcli counting cases: no device, nothing to claim
    timeout 120 python3 -I -c '
import fcntl, os, signal, struct, sys
d = "/sys/bus/usb/devices/" + sys.argv[1]
bus, dev = (int(open(d + f).read()) for f in ("/busnum", "/devnum"))
fd = os.open("/dev/bus/usb/%03d/%03d" % (bus, dev), os.O_RDWR)
fcntl.ioctl(fd, 0x8004550f, struct.pack("I", 1))  # USBDEVFS_CLAIMINTERFACE
print("held", flush=True)
signal.pause()
' "$EMU_BUSID" >"$TMP/gf-hold" 2>&1 &
    HOLDPID=$!
    i=0
    until grep -q '^held$' "$TMP/gf-hold"; do
        i=$((i + 1))
        if [ $i -gt 100 ] || ! kill -0 $HOLDPID 2>/dev/null; then
            emu_fail "greatfet: could not claim the interface: $(cat "$TMP/gf-hold")"
            return 1
        fi
        sleep 0.1
    done
}

gf_release() {
    kill $HOLDPID 2>/dev/null
    wait $HOLDPID 2>/dev/null
    HOLDPID=""
}

emu_cases() {
    F=greatfet
    # The address, in the text the harness's conn= rule does not reach.
    C='s/conn(=|: )[0-9]+\.[0-9]+/conn\1BUS.ADDR/g'
    Q="s/^'[0-9]+\\.[0-9]+'\$/'BUS.ADDR'/"
    # Stderr: oxrok's log prefix as sigrok's. A refused start: libsigrok's
    # session line, which has no counterpart, goes, and the last line is
    # sigrok-cli's for each.
    R='s/^oxrok: [a-z]+: oxrok::drivers::greatfet[a-z0-9_:]*: /sr: greatfet: /
/^sr: session: Could not start/d
s/^(device error: the capture did not start|device disconnected)$/Failed to start session./'
    # The C logs a failed stop once per transfer freed; once here.
    U='$!N; /^(.*)\n\1$/!P; D'
    # An unplug: the C can't send its stop, says so 16 times and meets the
    # gone device at the resubmit or in the callback by timing, with the
    # same effect; oxrok sends nothing to a board that left.
    D='/Cannot stop logic analyzer/d; /^oxrok: warn: oxrok::cli::pipeline: Device only sent/d; s/Cannot resubmit USB transfer\./Device gone during USB transfer./'
    # What the C fails to do and exits 0 on.
    W_STOP='after a failed transfer protocol.c:1097,968 send the stop from inside the transfer callback, where libusb_control_transfer() answers LIBUSB_ERROR_BUSY, so the board goes on sampling'
    W_LOST='protocol.c:961,978: the end of a capture runs at the first transfer freed and frees the feed queue unflushed, so the samples it holds are lost'
    W_OVER='protocol.c:1354-1358,1372-1373 cut the limit to whole bytes of sample memory and submit all the samples of the last'
    W_PACKED='protocol.c:1311 takes a feed of one byte a sample for whole samples and passes packed bytes on as they are'
    W_SINT='sigrok-cli has no SIGINT handler'
    W_WIDE='protocol.c:544 writes the u64 rate with write_u32le_inc(); the firmware field is uint32_t, so what goes out is another rate'
    W_SLOW='nothing in api.c or protocol.c doubts an answered rate below what the 12 bit divider reaches or over the clock'
    W_HELD='api.c:524 counts a failed submission as a start that never happened, so no stop follows the start that went through'
    # A long output: its first lines and the digest of all of it.
    LONG='1,12!d'
    A8=SGPIO0,SGPIO1,SGPIO2,SGPIO3,SGPIO4,SGPIO5,SGPIO6,SGPIO7
    B8=SGPIO8,SGPIO9,SGPIO10,SGPIO11,SGPIO12,SGPIO13,SGPIO14,SGPIO15

    gf_options
    emu_case "scan" --keep-header -- -d $F --scan
    emu_case "bad conn" --stderr --filter "$R
s/^sr: greatfet: /sr: usb: /" -- -d $F:conn=nonsense --scan
    emu_case "show" --keep-header --filter "$C" -- -d $F --show
    emu_case "show group" --keep-header --filter "$C" -- -d $F -g Logic --show
    emu_case "get conn" --filter "$Q" --keep-header -- -d $F --get conn
    emu_case "get samplerate" --keep-header -- -d $F --get samplerate
    emu_case "get continuous" --keep-header -- -d $F --get continuous
    emu_case "get limit_samples" --keep-header -- -d $F --get limit_samples
    emu_case "get limit_time" --keep-header -- -d $F --get limit_time
    emu_case "get set rate" --keep-header -- -d $F -c samplerate=1M --get samplerate
    emu_case "get unknown" --expect-fail --stderr --filter "$R" -- -d $F --get voltage_threshold
    emu_case "set rate" --no-output -- -d $F -c samplerate=2M --set
    emu_case "set limit" --no-output -- -d $F -c limit_samples=10 --set
    emu_case "set unknown" --expect-fail --stderr --filter "$R" -- -d $F -c voltage_threshold=1-2 --set
    emu_case "group get" --expect-fail --stderr --filter "$R
s/: not supported: not applicable/: not applicable/" -- -d $F -g Logic --get samplerate
    emu_case "group set" --expect-fail --stderr --filter "$R" -- -d $F -g Logic -c samplerate=1M --set

    # Selection.
    emu_case "by ids" --keep-header -- -d $F:conn=1d50.60e6 --scan
    emu_case "by address" --keep-header -- -d $F:conn=$EMU_USB --scan
    emu_case "other ids" --no-output -- -d $F:conn=1d50.60e7 --scan
    emu_case "sn whole" --keep-header -- -d $F:conn=sn=000000000000000057cc67e630187657 --scan
    emu_case "sn part" --keep-header -- -d $F:conn=sn=57cc67e6 --scan
    emu_case "sn other" --no-output -- -d $F:conn=sn=deadbeef --scan
    emu_case "sn capture" -- -d $F:conn=sn=30187657 --samples 16 -O bits
    gf_options stall_string=3
    emu_case "sn string stalled" --no-output -- -d $F:conn=sn=57cc67e6 --scan
    gf_options short_string=3
    emu_case "sn string short" --no-output -- -d $F:conn=sn=57cc67e6 --scan
    gf_options

    # Probe names: named channels pad up to 16 unless the list starts
    # with -, and then fewer than 9 make the feed one byte a sample.
    emu_case "probe names two" --keep-header -- -d $F:probe_names=CLK,DATA --scan
    emu_case "probe names two capture" -- -d $F:probe_names=CLK,DATA -C CLK,DATA --samples 24 -O bits
    emu_case "probe names nine" --keep-header -- -d $F:probe_names=a,b,c,d,e,f,g,h,i --scan
    emu_case "probe names many" --keep-header -- -d $F:probe_names=a,b,c,d,e,f,g,h,i,j,k,l,m,n,o,p,q,r --scan
    emu_case "probe names pad" --keep-header -- -d $F:probe_names=-a,b --scan
    emu_case "probe names dash eight capture" -- -d $F:probe_names=-a,b,c,d,e,f,g,h --samples 24 -O bits
    emu_case "probe names dash nine capture" -- -d $F:probe_names=-a,b,c,d,e,f,g,h,i --samples 24 -O bits
    emu_case "probe names dash two capture" --oracle-differs "$W_PACKED" -- -d $F:probe_names=-CLK,DATA --samples 24 -O bits
    emu_case "probe names dash one capture" --oracle-differs "$W_PACKED" -- -d $F:probe_names=-CLK --samples 24 -O bits
    emu_case "probe names dash four capture" --oracle-differs "$W_PACKED" -- -d $F:probe_names=-a,b,c,d --samples 24 -O bits

    # What the scan makes of the firmware's answers.
    gf_options stall=serial
    emu_case "serial stalled" --no-output -- -d $F --scan
    gf_options stall=version
    emu_case "version stalled" --no-output -- -d $F --scan
    gf_options stall-read=serial
    emu_case "serial read stalled" --no-output -- -d $F --scan
    gf_options stall-read=version
    emu_case "version read stalled" --no-output -- -d $F --scan
    gf_options serial=474653313233
    emu_case "serial as text" --keep-header -- -d $F --scan
    gf_options serial=
    emu_case "serial empty" --no-output -- -d $F --scan
    gf_options serial=0000000000000000
    emu_case "serial 8 bytes" --no-output -- -d $F --scan
    gf_options serial=00010000000000000000000000000000
    emu_case "serial word 0 not zero" --no-output -- -d $F --scan
    gf_options serial=00000000010000000000000000000000
    emu_case "serial word 1 not zero" --no-output -- -d $F --scan
    gf_options version=
    emu_case "version empty" --keep-header -- -d $F --scan
    gf_options version=00ff00
    emu_case "version binary" --keep-header -- -d $F --scan
    gf_options version=763230
    emu_case "version no NUL" --keep-header -- -d $F --scan
    gf_options

    # Rates: the firmware's divider answers, the driver's limit refuses.
    emu_case "default 8ch" -- -d $F --samples 1000 -O bits
    emu_case "rate 1M" -- -d $F -c samplerate=1M --samples 64 -O bits
    emu_case "rate 10M answers 10.2M" -- -d $F -c samplerate=10M --samples 64 -O bits
    emu_case "rate 40.8M" -- -d $F -c samplerate=40800k --samples 64 -O bits
    emu_case "rate 36M answers 40.8M" -- -d $F -c samplerate=36M --samples 64 -O bits
    emu_case "rate 42M answers 51M refused" --expect-fail --stderr --filter "$R" -- -d $F -c samplerate=42M --samples 64 -O bits
    emu_case "rate 51M refused" --expect-fail --stderr --filter "$R" -- -d $F -c samplerate=51M --samples 64 -O bits
    emu_case "rate 68M 4ch" -- -d $F -C SGPIO0,SGPIO1,SGPIO2,SGPIO3 -c samplerate=68M --samples 64 -O bits
    emu_case "rate 102M 4ch refused" --expect-fail --stderr --filter "$R" -- -d $F -C SGPIO0,SGPIO1,SGPIO2,SGPIO3 -c samplerate=102M --samples 64 -O bits
    emu_case "rate 72M 3ch refused" --expect-fail --stderr --filter "$R" -- -d $F -C SGPIO0,SGPIO1,SGPIO2 -c samplerate=72M --samples 64 -O bits
    emu_case "rate 102M 2ch" -- -d $F -C SGPIO0,SGPIO1 -c samplerate=102M --samples 64 -O bits
    emu_case "rate 204M 2ch refused" --expect-fail --stderr --filter "$R" -- -d $F -C SGPIO0,SGPIO1 -c samplerate=204M --samples 64 -O bits
    emu_case "rate 204M 1ch" -- -d $F -C SGPIO0 -c samplerate=204M --samples 64 -O bits
    emu_case "rate 204M upper 1ch" -- -d $F -C SGPIO8 -c samplerate=204M --samples 64 -O bits
    # Below what the 12 bit divider reaches: the firmware answers it all the
    # same, and oxrok warns (inferred, see the model).
    emu_case "rate 10k" -- -d $F -c samplerate=10k --samples 64 -O bits
    emu_case "rate 10k warns" --oracle-differs "$W_SLOW" --stderr --filter "$R" -- -d $F -c samplerate=10k --samples 64 -O bits
    # libsigrok's sr_config_set() refuses 0 before the driver.
    emu_case "rate 0 refused" --expect-fail --stderr --filter "$R
s/(invalid argument).*\$/\\1./" -- -d $F -c samplerate=0 --samples 64 -O bits
    # In the C the u64 setting goes out as 32 bits: 2^32 + 1 is 1 Hz, 2^32 is
    # 0, which the firmware takes for undivided. oxrok refuses them.
    emu_case "rate over 32 bits" --oracle-differs "$W_WIDE" --expect-fail --stderr --filter "$R" -- -d $F -c samplerate=4294967297 --samples 64 -O bits
    emu_case "rate 2^32" --oracle-differs "$W_WIDE" --expect-fail --stderr --filter "$R" -- -d $F -C SGPIO0 -c samplerate=4294967296 --samples 64 -O bits
    emu_case "rate 5G refused" --oracle-bug "$W_WIDE" --expect-fail --stderr --filter "$R" -- -d $F -c samplerate=5G --samples 64 -O bits

    # Channels: bus width and first pin, with and without gaps.
    emu_case "1ch" -- -d $F -C SGPIO0 --samples 104 -O bits
    emu_case "1ch limit in a byte" --oracle-differs "$W_OVER" -- -d $F -C SGPIO0 --samples 3 -O bits
    emu_case "2ch limit in a byte" --oracle-differs "$W_OVER" -- -d $F -C SGPIO0,SGPIO1 --samples 5 -O bits
    emu_case "2ch" -- -d $F -C SGPIO0,SGPIO1 --samples 100 -O bits
    emu_case "2ch second only" -- -d $F -C SGPIO1 --samples 100 -O bits
    emu_case "3ch" -- -d $F -C SGPIO0,SGPIO1,SGPIO2 --samples 100 -O bits
    emu_case "3ch third only" -- -d $F -C SGPIO2 --samples 100 -O bits
    emu_case "4ch" -- -d $F -C SGPIO0,SGPIO1,SGPIO2,SGPIO3 --samples 100 -O bits
    emu_case "5ch" -- -d $F -C SGPIO0,SGPIO1,SGPIO2,SGPIO3,SGPIO4 --samples 100 -O bits
    emu_case "6ch gap" -- -d $F -C SGPIO1,SGPIO5 --samples 100 -O bits
    emu_case "8ch last only" -- -d $F -C SGPIO7 --samples 100 -O bits
    emu_case "upper 1ch" -- -d $F -C SGPIO8 --samples 104 -O bits
    emu_case "upper 2ch" -- -d $F -C SGPIO8,SGPIO9 --samples 100 -O bits
    emu_case "upper 4ch" -- -d $F -C SGPIO8,SGPIO9,SGPIO10,SGPIO11 --samples 100 -O bits
    emu_case "upper 5ch gap" -- -d $F -C SGPIO12 --samples 100 -O bits
    emu_case "upper 8ch" -- -d $F -C $B8 --samples 100 -O bits
    emu_case "upper hex" -- -d $F -C $B8 --samples 64 -O hex
    emu_case "lower csv" --filter '/^;/d' -- -d $F -C SGPIO0,SGPIO1 --samples 16 -O csv:header=false

    # Pins held still: every transfer one byte repeated, small to record.
    gf_options level=a5c3
    emu_case "held 8ch" --record -- -d $F --samples 32 -O hex
    emu_case "held upper 4ch" --record -- -d $F -C SGPIO8,SGPIO9,SGPIO10,SGPIO11 --samples 32 -O bits
    emu_case "held 1ch" -- -d $F -C SGPIO0 --samples 16 -O bits
    gf_options

    # Both banks: 9 bits and more are refused by this firmware without
    # telling the host, so no samples come; wide=1 stands for one that can.
    emu_case "both banks no samples" --libsigrok --hangs --timeout 2 --stderr -- -d $F -C SGPIO0,SGPIO8 -c samplerate=1M --samples 8 -O bits
    emu_case "both banks oxrok waits" --oracle-bug "$W_SINT" --interrupt out=0:6 --no-output -- -d $F -C SGPIO0,SGPIO8 -c samplerate=1M --samples 8 -O bits
    emu_case "rate over the clock no samples" --libsigrok --hangs --timeout 2 --no-output -- -d $F -C SGPIO0 -c samplerate=205M --samples 8 -O bits
    # From the clock up to where the bandwidth limit refuses it: answered as
    # asked, no samples, and oxrok warns.
    emu_case "rate over the clock oxrok waits" --oracle-bug "$W_SINT" --interrupt out=0:6 --stderr --filter "$R" -- -d $F -C SGPIO0 -c samplerate=205M --samples 8 -O bits
    emu_case "rate 336M no samples" --libsigrok --hangs --timeout 2 --no-output -- -d $F -C SGPIO0 -c samplerate=336M --samples 8 -O bits
    gf_options wide=1
    emu_case "wide 16ch" --stderr --filter "$R" -- -d $F -C $A8,$B8 -c samplerate=20400k --samples 8 -O bits
    emu_case "wide 16ch output" -- -d $F -C $A8,$B8 -c samplerate=20400k --samples 80 -O bits
    emu_case "wide 2 banks" -- -d $F -C SGPIO0,SGPIO8 -c samplerate=1M --samples 40 -O bits
    emu_case "wide 16ch refused" --expect-fail --stderr --filter "$R" -- -d $F -C $A8,$B8 -c samplerate=25500k --samples 8 -O bits
    gf_options wide=1 in-limit=3
    emu_case "wide 3 transfers" --digest --filter "$LONG" -- -d $F -C $A8,$B8 -c samplerate=1M --samples 300000 -O bits
    gf_options

    # Several transfers, and limits on their boundaries.
    gf_options in-limit=1
    emu_case "whole transfer" --digest --filter "$LONG" -- -d $F --samples 262144 -O bits
    gf_options in-limit=2
    emu_case "one sample over" --digest --filter "$LONG" -- -d $F --samples 262145 -O bits
    gf_options in-limit=3
    emu_case "3 transfers" --digest --filter "$LONG" -- -d $F --samples 786000 -O bits
    gf_options in-limit=2
    emu_case "2 transfers 1ch" --digest --filter "$LONG" -- -d $F -C SGPIO0 --samples 2097160 -O bits
    emu_case "2 transfers 2ch" --digest --filter "$LONG" -- -d $F -C SGPIO1 --samples 1048580 -O bits
    gf_options in-limit=2
    emu_case "2 transfers upper 8ch" --digest --filter "$LONG" -- -d $F -C $B8 --samples 500000 -O bits
    gf_options

    # Time limits. No samples come, or one transfer's; the time runs out
    # well after that transfer, at the driver's next check.
    gf_options in-limit=0
    emu_case "time no samples" --no-output -- -d $F --time 100 -O bits
    gf_options in-limit=1
    emu_case "time one transfer" --digest --filter "$LONG" -- -d $F --time 300 -O bits
    gf_options

    # A stop by hand: oxrok on SIGINT, the library from its main loop.
    gf_options in-limit=0
    emu_case "interrupt no samples" --oracle-bug "$W_SINT" --interrupt out=0:6 --no-output -- -d $F --continuous -O bits
    gf_options in-limit=1
    emu_case "interrupt with samples" --oracle-bug "$W_SINT" --interrupt in=1:1 --digest --filter "$LONG" -- -d $F --continuous -O bits
    gf_options in-limit=0
    emu_case "lib stop at the header" --libsigrok --no-output -- -d $F --stop-after 0 -O bits
    emu_case "lib stop after time" --libsigrok --no-output -- -d $F --stop-after-ms 200 -O bits
    gf_options in-limit=1
    emu_case "lib stop after time with samples" --libsigrok --digest --filter "$LONG" -- -d $F --stop-after-ms 300 -O bits
    # The feed queue holds 4 MiB samples: the 16th transfer flushes it,
    # which is the runner's first look at any sample.
    gf_options in-limit=16
    emu_case "lib stop after samples" --libsigrok -- -d $F --stop-after 40 -O bits
    gf_options in-limit=1
    emu_case "lib limit" --libsigrok -- -d $F --samples 40 -O bits
    emu_case "lib scan" --libsigrok --keep-header -- -d $F --scan
    gf_options

    # A capture the firmware fails.
    gf_options stall=first-pin
    emu_case "first pin stalled" --expect-fail --stderr --filter "$R" -- -d $F --samples 8 -O bits
    gf_options stall=alt
    emu_case "alt stalled" --expect-fail --stderr --filter "$R" -- -d $F --samples 8 -O bits
    gf_options stall=configure
    emu_case "configure stalled" --expect-fail --stderr --filter "$R" -- -d $F --samples 8 -O bits
    gf_options stall-read=configure
    emu_case "configure read stalled" --expect-fail --stderr --filter "$R" -- -d $F --samples 8 -O bits
    gf_options configure-len=0
    emu_case "configure empty" --expect-fail --stderr --filter "$R" -- -d $F --samples 8 -O bits
    gf_options configure-len=8
    emu_case "configure short" --expect-fail --stderr --filter "$R" -- -d $F --samples 8 -O bits
    gf_options stall=start
    emu_case "start stalled" --expect-fail --stderr --filter "$R" -- -d $F --samples 8 -O bits
    gf_options stall=stop
    emu_case "stop stalled" --stderr --filter "$R
$U" -- -d $F --samples 8 -O bits
    emu_case "stop stalled output" -- -d $F --samples 8 -O bits
    gf_options

    # The firmware's overrun: a halted endpoint and no more stream.
    gf_options in-stall=0 in-limit=2
    # The transactions alone differ: the first of these flips if the C's stop
    # ever goes out. The second pins oxrok's message.
    emu_case "halted at once" --oracle-differs "$W_STOP" --no-output -- -d $F --samples 8 -O bits
    emu_case "halted at once message" --oracle-differs "$W_STOP" --stderr --filter "$R" -- -d $F --samples 8 -O bits
    # The C's stop as it is, failed from inside the callback, 16 times.
    emu_case "lib halted at once" --libsigrok --stderr -- -d $F --samples 8 -O bits
    gf_options in-stall=1 in-limit=2
    emu_case "halted after one" --oracle-differs "$W_LOST" --digest --filter "$LONG" -- -d $F --samples 300000 -O bits
    gf_options

    # A cable pulled during a capture.
    gf_options in-limit=2
    emu_case "unplug" --unplug in=1:1 --stderr --filter "$R
$D" -- -d $F --samples 300000 -O bits
    gf_options in-limit=0
    emu_case "unplug at start" --unplug out=0:6 --stderr --filter "$R
$D" -- -d $F --samples 8 -O bits
    emu_case "unplug at configure" --expect-fail --unplug out=0:5 --stderr --filter "$R" -- -d $F --samples 8 -O bits
    gf_options

    # Another program has the samples interface: libusb's claim fails,
    # the driver goes on, and the first submission fails.
    gf_hold
    emu_case "samples interface held" --oracle-bug "$W_HELD" --expect-fail --stderr --filter "$R" -- -d $F --samples 8 -O bits
    gf_release
}
