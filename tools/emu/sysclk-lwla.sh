# The Sysclk LWLA1034 and LWLA1016 (emu/src/models/sysclk-lwla/): an
# FX2 with its firmware in EEPROM, 2961:6689 and 2961:6688, relaying
# word commands to an FPGA that takes its bitstream on every open.
# Sourced by tools/diffcli.sh --emu; see docs/USB.md, "Writing a device
# model". oxrok has the driver (src/drivers/sysclk_lwla): every case runs
# both tools.
EMU_MODEL=sysclk-lwla
EMU_DRIVER=sysclk-lwla
EMU_FULL=sysclk-lwla
EMU_GCOV_SHARED="src/usb.c"

# The model's options for the cases that follow (see fx2lafw.sh).
lwla_options() {
    set -- options "$@"
    emu_cmd "$*" >/dev/null || emu_fail "sysclk-lwla: the emulator refused '$*'"
}

# Claims interface 0 through usbfs as another program would, until
# lwla_release (as h4032l_hold in hantek-4032l.sh).
lwla_hold() {
    [ "$EMU_DRY" = 0 ] || return 0 # diffcli counting cases: no device, nothing to claim
    timeout 120 python3 -I -c '
import fcntl, os, signal, struct, sys
d = "/sys/bus/usb/devices/" + sys.argv[1]
bus, dev = (int(open(d + f).read()) for f in ("/busnum", "/devnum"))
fd = os.open("/dev/bus/usb/%03d/%03d" % (bus, dev), os.O_RDWR)
fcntl.ioctl(fd, 0x8004550f, struct.pack("I", 0))  # USBDEVFS_CLAIMINTERFACE
print("held", flush=True)
signal.pause()
' "$EMU_BUSID" >"$TMP/lwla-hold" 2>&1 &
    HOLDPID=$!
    i=0
    until grep -q '^held$' "$TMP/lwla-hold"; do
        i=$((i + 1))
        if [ $i -gt 100 ] || ! kill -0 $HOLDPID 2>/dev/null; then
            emu_fail "sysclk-lwla: could not claim the interface: $(cat "$TMP/lwla-hold")"
            return 1
        fi
        sleep 0.1
    done
}

lwla_release() {
    kill $HOLDPID 2>/dev/null
    wait $HOLDPID 2>/dev/null
    HOLDPID=""
}

# Moves a bitstream stand-in away for a case (lwla_restore puts it
# back), or writes a broken one: lwla_bitstream NAME empty|huge.
lwla_bitstream() {
    [ "$EMU_DRY" = 0 ] || return 0 # diffcli counting cases: nothing to move
    cp "$SIGROK_FIRMWARE_DIR/$1" "$TMP/lwla-saved" || return 1
    case $2 in
        missing) rm -f "$SIGROK_FIRMWARE_DIR/$1" ;;
        empty) : >"$SIGROK_FIRMWARE_DIR/$1" ;;
        huge) head -c 262145 /dev/zero >"$SIGROK_FIRMWARE_DIR/$1" ;;
    esac
}

lwla_restore() {
    [ "$EMU_DRY" = 0 ] || return 0
    mv "$TMP/lwla-saved" "$SIGROK_FIRMWARE_DIR/$1" ||
        emu_fail "sysclk-lwla: could not put $1 back"
}

emu_cases() {
    F=sysclk-lwla
    # oxrok's log prefix as libsigrok's, and the two messages libsigrok's
    # own layers log (usb.c, resource.c) with their prefix; the tool's
    # last line without oxrok's added reason. The filters below read this
    # one form.
    N='s/^oxrok: [a-z]+: oxrok::drivers::sysclk_lwla(::protocol)?: /sr: sysclk-lwla: /; s/^sr: sysclk-lwla: (Could neither|Invalid input)/sr: usb: \1/; s/^sr: sysclk-lwla: (Failed to open resource)/sr: resource: \1/; s/^Failed to open device: .*$/Failed to open device./'
    # The driver's own messages, without the timestamps of -l 5. Dropped:
    # device.c's "Opening/Closing device instance", which is libsigrok's
    # core tagged with the driver's name, not the driver.
    E="$N; /sysclk-lwla:/!d; s/ *\[[0-9:.]+\]//; /^sr: device: /d"
    # The driver's and sigrok-cli's errors (the open's failure). Dropped
    # as well: session.c's "Failed to commit", libsigrok's core saying what
    # the driver said on the line before.
    O="$N; /sysclk-lwla:|Failed/!d; s/ *\[[0-9:.]+\]//; /^sr: (session|device): /d"
    # The debug line of a failing request and the open's retries.
    X="$N; /sysclk-lwla: (Failed|Invalid|Downloading)/!d; s/ *\[[0-9:.]+\]//"
    # A refused option: sr_strerror()'s wording for oxrok's errors. It
    # drops the reason oxrok adds, not the refusal.
    A='s/invalid argument: samplerate\.$/invalid samplerate./; s/invalid argument: .*$/invalid argument./; s/: not supported\.$/: not applicable./'
    # A conn= that selects nothing reads as no conn= at all.
    C='api.c:104 reads an empty conn_matches as no conn=, so a conn= that selects nothing lists every LWLA'
    # Ctrl-C: sigrok-cli has no SIGINT handler and dies (not a driver bug,
    # but the form the harness has for it); oxrok ends the capture. The
    # "lib" cases show what libsigrok's driver does there.
    I='sigrok-cli has no SIGINT handler and dies where oxrok ends the capture'
    # A capture is held to its first lines and the digest of the whole.
    B='1,4!d'

    # --- LWLA1034 ---
    lwla_options
    emu_case "scan" --keep-header -- -d $F --scan
    emu_case "by ids" -- -d $F:conn=2961.6689 --scan
    emu_case "by address" -- -d $F:conn=$EMU_USB --scan
    # sr_usb_find() matches nothing: no match list, so every LWLA is
    # listed (api.c:104). oxrok lists what was selected: nothing.
    emu_case "conn matching nothing" --oracle-differs "$C" --no-output -- -d $F:conn=9.99 --scan
    # sr_usb_find() refuses the spec; the scan lists the board all the
    # same.
    emu_case "bad conn" --stderr --filter "$N; /usb:|sysclk-lwla:/!d; s/ *\[[0-9:.]+\]//" -- -l 4 -d $F:conn=nonsense --scan
    emu_case "bad conn lists" --oracle-differs "$C" --no-output -- -d $F:conn=nonsense --scan
    # The root hub of the emulator's bus: matched, and not an LWLA. Library
    # only: nusb does not list root hubs, so oxrok finds nothing there, and
    # this bus has no other device to name.
    emu_case "lib conn another device" --libsigrok --stderr --filter "$E; s/device [0-9]+\.1 /device BUS.1 /" -- -l 4 -d $F:conn=${EMU_USB%.*}.1 --scan
    emu_case "show" --keep-header -- -d $F --show
    emu_case "open and close" --stderr --filter "$E" -- -l 5 -d $F --show
    for k in samplerate limit_samples limit_time external_clock clock_edge triggersource triggerslope; do
        emu_case "get $k" -- -d $F --get $k
    done
    emu_case "get rle refused" --expect-fail --stderr -- -d $F --get rle
    emu_case "set samplerate" --no-output -- -d $F --config samplerate=1M --set
    emu_case "set external clock" --no-output -- -d $F --config external_clock=1:clock_edge=f --set

    emu_case "100 samples 125M" --digest --filter "$B" -- -d $F --samples 100 -O bits
    emu_case "100M" --digest --filter "$B" -- -d $F -C CH1,CH2,CH3,CH4 -c samplerate=100M --samples 1000 -O bits
    emu_case "1M hex" --digest --filter "$B" -- -d $F -c samplerate=1M --samples 500 -O hex
    emu_case "100 Hz" --record --digest -- -d $F -C CH1,CH2,CH3 -c samplerate=100 --samples 25 -O bits
    emu_case "time 300 ms" --digest --filter "$B" -- -l 5 -d $F -C CH1,CH4 -c samplerate=1k --time 300 -O bits
    emu_case "time spew" --stderr --filter "$N; /Sampling in progress|Time limit|words in capture|Acquisition/!d; s/ *\[[0-9:.]+\]//" -- -l 5 -d $F -C CH1,CH4 -c samplerate=1k --time 300 -O bits
    # A time limit shorter than a sample: samples_max is 0
    # (protocol.c:510), so the readout passes on nothing.
    emu_case "time shorter than a sample" --no-output -- -d $F -C CH1 -c samplerate=100 --time 5 -O bits
    emu_case "30000 samples" --digest --filter "$B" -- -d $F -C CH1,CH2 -c samplerate=10M --samples 30000 -O bits
    emu_case "run of two" --digest --filter "$B" -- -d $F -C CH2 -c samplerate=1M --samples 200 -O bits
    emu_case "runs with length" --digest --filter "$B" -- -d $F -C CH3,CH5 -c samplerate=1M --samples 300 -O bits
    emu_case "110M boosts" --digest --filter "$B" -- -d $F -C CH1 -c samplerate=110M --samples 100 -O bits
    emu_case "60M runs at 100M" --digest --filter "$B" -- -d $F -C CH1 -c samplerate=60M --samples 100 -O bits
    emu_case "3M divides" --digest --filter "$B" -- -d $F -C CH1 -c samplerate=3M --samples 100 -O bits
    emu_case "csv 34 channels" --digest --filter '1,3!d' -- -d $F -c samplerate=20M --samples 64 -O csv:header=false
    lwla_options period=5
    emu_case "period 5" --digest --filter "$B" -- -d $F -C CH1,CH2 -c samplerate=1M --samples 100 -O bits

    lwla_options
    emu_case "trigger high" --digest --filter "$B" -- -d $F -C CH1,CH2,CH3 -t CH3=1 -c samplerate=1M --samples 64 -O bits
    emu_case "trigger rising" --digest --filter "$B" -- -d $F -C CH1,CH2,CH3,CH4 -t CH4=r -c samplerate=1M --samples 64 -O bits
    emu_case "trigger falling and low" --digest --filter "$B" -- -d $F -C CH1,CH2,CH3,CH4 -t CH2=f,CH4=0 -c samplerate=1M --samples 64 -O bits
    emu_case "trigger late" --digest --filter "$B" -- -d $F -C CH1,CH20 -t CH20=r -c samplerate=1M --samples 64 -O bits
    # Both wait for a trigger that never comes, until they are stopped.
    # Oxrok's stop writes the whole stop sequence. The status polls of the
    # hang are not cut: the interrupt comes after the third (replies 8-10).
    emu_case "trigger never fires" --interrupt in=6:10 --oracle-bug "$I" --no-output -- -d $F -C CH1,CH2 -t CH1=r,CH2=r -c samplerate=1k --samples 64 -O bits
    emu_case "lib trigger never fires" --libsigrok --hangs --timeout 2 --no-output -- -d $F -C CH1,CH2 -t CH1=r,CH2=r -c samplerate=1k --samples 64 -O bits
    # One sequence of the trigger, as sigrok-cli hands it over: two stages.
    emu_case "two trigger stages" --expect-fail --stderr --filter "$O" -- -d $F -C CH1 -t CH1=01 --samples 10 -O bits
    # A stop: with the C driver, only the first write of each sequence
    # goes out (protocol.c:341), here the long register's address and not
    # the strobe, so the capture runs on; see "trigger never fires" and
    # "interrupt while sampling" for oxrok's.
    emu_case "interrupt while sampling" --interrupt in=6:9 --oracle-bug "$I" --no-output -- -d $F -C CH1,CH4 -c samplerate=1k --time 100s -O bits
    emu_case "lib stop at the header" --libsigrok --no-output -- --stop-after 0 -d $F -C CH1 -c samplerate=1k --samples 64 -O bits
    # From the status wait (protocol.c:293), halfway between the first poll
    # at 100 ms and the second: the stop's first write alone.
    emu_case "lib stop from the status wait" --libsigrok --no-output -- --stop-after-ms 150 -d $F -C CH1,CH4 -c samplerate=1k --time 100s -O bits
    emu_case "lib stop during the readout" --libsigrok --digest --filter "$B" -- --stop-after 100 -d $F -C CH1,CH2 -c samplerate=10M --samples 30000 -O bits
    emu_case "trigger TRG rising" --digest --filter "$B" -- -d $F -C CH1,CH2 -c samplerate=1M:triggersource=TRG:triggerslope=r --samples 64 -O bits
    emu_case "trigger TRG falling" --digest --filter "$B" -- -d $F -C CH1,CH2 -c samplerate=1M:triggersource=TRG:triggerslope=f --samples 64 -O bits
    emu_case "trigger TRG and a level" --digest --filter "$B" -- -d $F -C CH1,CH12 -t CH12=1 -c samplerate=1M:triggersource=TRG:triggerslope=r --samples 64 -O bits

    # The external clock: no time limit is derived from a sample limit
    # (protocol.c:499-519), so the capture runs until the memory fills.
    emu_case "external clock" --digest --filter "$B" -- -d $F -C CH1,CH2 -c external_clock=1 --samples 1000 -O bits
    emu_case "external clock falling" --digest --filter "$B" -- -d $F -C CH1,CH2 -c external_clock=1:clock_edge=f --samples 1000 -O bits
    lwla_options ext_clock=10000
    # A time limit alone: no sample limit either, so the whole memory is
    # read (protocol.c:177, 228-242).
    emu_case "external clock time" --digest --filter "$B" -- -d $F -C CH1 -c external_clock=1 --time 100 -O bits

    lwla_options
    emu_case "samplerate too low" --expect-fail --stderr --filter "$A" -- -d $F -c samplerate=50 --samples 10
    emu_case "samplerate too high" --expect-fail --stderr --filter "$A" -- -d $F -c samplerate=200M --samples 10
    emu_case "time too long" --expect-fail --stderr -- -d $F --time 1000001s
    emu_case "samples too many" --expect-fail --stderr -- -d $F --samples 2T
    emu_case "clock edge refused" --expect-fail --stderr --filter "$A" -- -d $F -c clock_edge=x --samples 10
    emu_case "trigger source refused" --expect-fail --stderr --filter "$A" -- -d $F -c triggersource=x --samples 10
    emu_case "trigger slope refused" --expect-fail --stderr --filter "$A" -- -d $F -c triggerslope=x --samples 10
    emu_case "rle refused" --expect-fail --stderr --filter "$A" -- -d $F -c rle=1 --samples 10 -O bits
    emu_case "continuous refused" --expect-fail --stderr -- -d $F --continuous

    # The FX2 firmware's short transfer quirk.
    lwla_options short_transfers=1
    emu_case "short transfers" --digest --filter "$B" -- -d $F -C CH1,CH2 -c samplerate=1M --samples 100 -O bits
    emu_case "short transfers warning" --stderr --filter "$E" -- -d $F -C CH1 -c samplerate=1M --samples 10 -O bits

    # Faults of the open. The Nth command of a kind counts from the
    # model's start; an open goes: write-reg 1, read-reg 1-3 (the
    # ignored first test read: strobe, high, low), write-reg 2, read-reg
    # 4-6 (the second), read-lregs 1 (the quirk check), and tries the
    # whole open three times (api.c:233).
    lwla_options stale=100
    emu_case "fault stale reply" --stderr --filter "$E" -- -d $F -C CH1 -c samplerate=1M --samples 10 -O bits
    lwla_options halt_in=1
    emu_case "fault reply endpoint halted" --expect-fail --stderr --filter "$O" -- -d $F -C CH1 --samples 10 -O bits
    lwla_options stall=bitstream:1
    emu_case "fault bitstream once" --stderr --filter "$E" -- -d $F -C CH1 -c samplerate=1M --samples 10 -O bits
    lwla_options stall=bitstream
    emu_case "fault bitstream always" --expect-fail --stderr --filter "$O" -- -d $F -C CH1 --samples 10 -O bits
    # The first test read's errors are ignored (lwla1034.c:339).
    lwla_options stall=read-reg:2
    emu_case "fault first test read ignored" -- -d $F -C CH1 -c samplerate=1M --samples 10 -O bits
    lwla_options stall=read-reg:5
    emu_case "fault second test read" --stderr --filter "$X" -- -l 5 -d $F -C CH1 -c samplerate=1M --samples 10 -O bits
    lwla_options short_reply=read-reg:5
    emu_case "fault short register read" --stderr --filter "$N; /Invalid register|Downloading/!d; s/ *\[[0-9:.]+\]//" -- -l 5 -d $F -C CH1 -c samplerate=1M --samples 10 -O bits
    lwla_options test_id=0x1
    emu_case "fault test word" --expect-fail --stderr --filter "$O" -- -d $F -C CH1 --samples 10 -O bits
    # The second test read's other steps: its address write, its strobe
    # and its low word.
    for f in write-reg:2 read-reg:4 read-reg:6; do
        lwla_options stall=$f
        emu_case "fault second test read $f" --stderr --filter "$X" -- -l 5 -d $F -C CH1 -c samplerate=1M --samples 10 -O bits
    done
    lwla_options stall_reply=read-lregs:1
    emu_case "fault quirk check reply" --stderr --filter "$X" -- -l 5 -d $F -C CH1 -c samplerate=1M --samples 10 -O bits
    # Unplugged after the first 64 bytes of the quirk check's reply (the
    # 7th reply transfer): the drain of its tail fails, and the open's
    # second try finds no device (api.c:234-236).
    lwla_options short_transfers=1
    emu_case "unplug in the quirk check" --unplug in=6:7 --expect-fail --stderr --filter "$X" -- -l 5 -d $F -C CH1 --samples 10 -O bits
    lwla_options stall=read-lregs:1
    emu_case "fault quirk check" --stderr --filter "$X" -- -l 5 -d $F -C CH1 -c samplerate=1M --samples 10 -O bits
    lwla_options short_reply=read-lregs:1
    emu_case "fault quirk check short" --stderr --filter "$E" -- -d $F -C CH1 -c samplerate=1M --samples 10 -O bits
    lwla_options short_transfers=1 short_reply=read-lregs:1
    emu_case "fault quirk check split short" --stderr --filter "$E" -- -d $F -C CH1 -c samplerate=1M --samples 10 -O bits
    # The close's shut-down bitstream (the second upload), after --show:
    # sr_dev_close() sets the status INACTIVE first. A close from
    # sr_exit() goes through std_dev_clear() with the status still ACTIVE,
    # so the FPGA stays on after a capture.
    lwla_options stall=bitstream:2
    emu_case "fault shut down" --stderr --filter "$E" -- -d $F --show
    # The commit's bitstream for the external clock (the second upload).
    emu_case "fault commit bitstream" --expect-fail --stderr --filter "$O" -- -d $F -C CH1 -c external_clock=1 --samples 10 -O bits
    # emu_case begins with a reset, which re-attaches only when the
    # descriptors change: every case of this section is an LWLA1034, so
    # the claim stays on the attached device.
    if lwla_hold; then
        emu_case "interface claimed by another program" --expect-fail --stderr --filter "$O" -- -d $F -C CH1 --samples 10 -O bits
        lwla_release
    fi

    # Faults of the capture: write-reg 3-8 (capture_init), 9 (boost),
    # write-lregs 1, then, asynchronous, write-reg 10-13 (the start),
    # read-lregs 2 (the first status poll), write-reg 14-18 (the stop),
    # read-reg 7 (the length), write-reg 19-21 (the readout's prepare),
    # read-mem 1..., write-reg 22 (its end).
    for f in write-reg:3 write-reg:9 write-lregs:1; do
        lwla_options stall=$f
        emu_case "fault setup $f" --expect-fail --stderr --filter "$O" -- -d $F -C CH1 -c samplerate=1M --samples 10 -O bits
    done
    for f in stall=write-reg:10 stall=read-lregs:2 stall_reply=read-lregs:2 short_reply=read-lregs:2 stall=write-reg:14 stall=read-reg:7 stall_reply=read-reg:7 short_reply=read-reg:7 stall=write-reg:19 stall=read-mem:1 stall_reply=read-mem:1 short_reply=read-mem:1 stall=write-reg:22; do
        lwla_options $f
        emu_case "fault capture $f" --stderr --filter "$E" -- -d $F -C CH1 -c samplerate=1M --samples 300 -O bits
    done
    # The 100th of the 134 reads fails: the C has sent its first packet of
    # 20000 samples and drops the 2000 or so decoded since (protocol.c:216-219),
    # oxrok passes them on. The first line of the golden is the number of
    # lines of the output, which shows it.
    lwla_options stall=read-mem:100
    emu_case "fault later read keeps the samples" --oracle-differs "protocol.c:216-219 returns with the partly filled packet unsent" --digest --filter '$=; d' -- -d $F -C CH1,CH2 -c samplerate=10M --samples 30000 -O bits
    # Unplugged after the first status reply (the 8th reply).
    lwla_options
    emu_case "unplug while polling" --unplug in=6:8 --stderr --filter "$E" -- -d $F -C CH1,CH4 -c samplerate=1k --time 300 -O bits

    # Broken bitstream files.
    for how in missing empty huge; do
        if lwla_bitstream sysclk-lwla1034-int.rbf $how; then
            emu_case "bitstream file $how" --expect-fail --stderr --filter "$O" -- -d $F -C CH1 --samples 10 -O bits
            lwla_restore sysclk-lwla1034-int.rbf
        fi
    done

    # --- LWLA1016 ---
    lwla_options model=1016
    emu_case "1016 scan" --keep-header -- -d $F --scan
    emu_case "1016 show" --keep-header -- -d $F --show
    emu_case "1016 get rle" -- -d $F --get rle
    emu_case "1016 get external clock refused" --expect-fail --stderr -- -d $F --get external_clock
    emu_case "1016 100M" --digest --filter "$B" -- -d $F --samples 100 -O bits
    emu_case "1016 1M plain" --digest --filter "$B" -- -d $F -C CH1,CH2,CH3 -c samplerate=1M:rle=0 --samples 200 -O bits
    # One sample at 100 Hz in 10 ms does not fill a word of two: the
    # capture has no words (protocol.c:177-179) and nothing is read.
    lwla_options model=1016 tick=10
    emu_case "1016 capture with no words" --no-output -- -d $F -C CH1 -c samplerate=100:rle=0 --time 1 -O bits
    lwla_options model=1016
    emu_case "1016 odd samples plain" --digest --filter "$B" -- -d $F -c samplerate=1M:rle=0 --samples 101 -O bits
    emu_case "1016 rle runs" --record --digest --filter "$B" -- -d $F -C CH5,CH6 -c samplerate=1M --samples 300 -O bits
    emu_case "1016 30000 samples" --digest --filter "$B" -- -d $F -C CH1 -c samplerate=10M:rle=0 --samples 60000 -O bits
    emu_case "1016 time" --digest --filter "$B" -- -d $F -C CH1,CH2 -c samplerate=1k --time 300 -O bits
    emu_case "1016 trigger" --digest --filter "$B" -- -d $F -C CH1,CH2,CH3 -t CH3=r -c samplerate=1M --samples 64 -O bits
    # The third poll's last reply: 4 of the open's and 9 of the polls'.
    emu_case "1016 trigger never fires" --interrupt in=6:13 --oracle-bug "$I" --no-output -- -d $F -C CH1,CH2 -t CH1=r,CH2=r -c samplerate=1k --samples 64 -O bits
    emu_case "1016 lib trigger never fires" --libsigrok --hangs --timeout 2 --no-output -- -d $F -C CH1,CH2 -t CH1=r,CH2=r -c samplerate=1k --samples 64 -O bits
    emu_case "1016 lib stop from the status wait" --libsigrok --no-output -- --stop-after-ms 150 -d $F -C CH1,CH2 -c samplerate=1k --time 100s -O bits
    emu_case "1016 lib stop at the header" --libsigrok --no-output -- --stop-after 0 -d $F -C CH1 -c samplerate=1k --samples 64 -O bits
    emu_case "1016 3M divides" --digest --filter "$B" -- -d $F -C CH1 -c samplerate=3M:rle=0 --samples 100 -O bits
    lwla_options model=1016 short_transfers=1
    emu_case "1016 short transfers" --digest --filter "$B" -- -d $F -C CH1,CH2 -c samplerate=1M --samples 100 -O bits

    # LWLA1016 faults. The open: read-reg 1 (the ignored test read),
    # read-reg 2, write-reg 1-2 (the memory reset), read-mem 1-2 (the
    # quirk checks). A capture: write-reg 3 (channels), 4 (divider), 5-13
    # (capture_init), 14 (trigger), 15 (the start), read-reg 3-5 per
    # status poll, write-reg 16-17 (the stop), the length read, write-reg
    # 18 (read mode), read-mem 3..., write-reg 19-20.
    lwla_options model=1016 stall_reply=read-reg:1
    emu_case "1016 fault first test read ignored" -- -d $F -C CH1 -c samplerate=1M --samples 10 -O bits
    lwla_options model=1016 test_id=0x1
    emu_case "1016 fault test word" --expect-fail --stderr --filter "$O" -- -d $F -C CH1 --samples 10 -O bits
    lwla_options model=1016 stall=read-mem:1
    emu_case "1016 fault quirk check" --stderr --filter "$X" -- -l 5 -d $F -C CH1 -c samplerate=1M --samples 10 -O bits
    lwla_options model=1016 stall_reply=read-mem:1
    emu_case "1016 fault quirk check reply" --stderr --filter "$X" -- -l 5 -d $F -C CH1 -c samplerate=1M --samples 10 -O bits
    lwla_options model=1016 stall=read-reg:2
    emu_case "1016 fault second test read" --stderr --filter "$X" -- -l 5 -d $F -C CH1 -c samplerate=1M --samples 10 -O bits
    # Unplugged after the first 64 bytes of the first test read (the 3rd
    # reply transfer).
    lwla_options model=1016 short_transfers=1
    emu_case "1016 unplug in the quirk check" --unplug in=6:3 --expect-fail --stderr --filter "$X" -- -l 5 -d $F -C CH1 --samples 10 -O bits
    lwla_options model=1016 short_reply=read-mem:1
    emu_case "1016 fault quirk check short" --stderr --filter "$E" -- -d $F -C CH1 -c samplerate=1M --samples 10 -O bits
    lwla_options model=1016 short_transfers=1 short_reply=read-mem:1
    emu_case "1016 fault quirk check split short" --stderr --filter "$E" -- -d $F -C CH1 -c samplerate=1M --samples 10 -O bits
    lwla_options model=1016 stall=write-reg:1
    emu_case "1016 fault memory reset" --stderr --filter "$X" -- -l 5 -d $F -C CH1 -c samplerate=1M --samples 10 -O bits
    for f in write-reg:3 write-reg:4 write-reg:5 write-reg:14; do
        lwla_options model=1016 stall=$f
        emu_case "1016 fault setup $f" --expect-fail --stderr --filter "$O" -- -d $F -C CH1 -c samplerate=1M --samples 10 -O bits
    done
    for f in stall=write-reg:15 stall=read-reg:3 stall_reply=read-reg:4 short_reply=read-reg:3 stall=write-reg:16 short_reply=read-mem:3 stall=read-mem:3; do
        lwla_options model=1016 $f
        emu_case "1016 fault capture $f" --stderr --filter "$E" -- -d $F -C CH1 -c samplerate=1M --samples 300 -O bits
    done
}
