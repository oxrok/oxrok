# The Sysclk SLA5032 (emu/src/models/sysclk-sla5032.rs): an FX2LP with its
# firmware in EEPROM, 2961:66b0, relaying commands to an FPGA that takes
# its bitstream once per power-up and needs keys from the EEPROM.
# Sourced by tools/diffcli.sh --emu; see docs/USB.md, "Writing a device
# model". oxrok has the driver (src/drivers/sysclk_sla5032): every case
# runs both tools.
EMU_MODEL=sysclk-sla5032
EMU_DRIVER=sysclk-sla5032
EMU_FULL=sysclk-sla5032
EMU_GCOV_SHARED="src/usb.c"

# The model's options for the cases that follow (see fx2lafw.sh).
sla_options() {
    set -- options "$@"
    emu_cmd "$*" >/dev/null || emu_fail "sysclk-sla5032: the emulator refused '$*'"
}

# Claims interface 0 through usbfs as another program would, until
# sla_release (as lwla_hold in sysclk-lwla.sh).
sla_hold() {
    [ "$EMU_DRY" = 0 ] || return 0 # diffcli counting cases: no device, nothing to claim
    timeout 120 python3 -I -c '
import fcntl, os, signal, struct, sys
d = "/sys/bus/usb/devices/" + sys.argv[1]
bus, dev = (int(open(d + f).read()) for f in ("/busnum", "/devnum"))
fd = os.open("/dev/bus/usb/%03d/%03d" % (bus, dev), os.O_RDWR)
fcntl.ioctl(fd, 0x8004550f, struct.pack("I", 0))  # USBDEVFS_CLAIMINTERFACE
print("held", flush=True)
signal.pause()
' "$EMU_BUSID" >"$TMP/sla-hold" 2>&1 &
    HOLDPID=$!
    i=0
    until grep -q '^held$' "$TMP/sla-hold"; do
        i=$((i + 1))
        if [ $i -gt 100 ] || ! kill -0 $HOLDPID 2>/dev/null; then
            emu_fail "sysclk-sla5032: could not claim the interface: $(cat "$TMP/sla-hold")"
            return 1
        fi
        sleep 0.1
    done
}

sla_release() {
    kill $HOLDPID 2>/dev/null
    wait $HOLDPID 2>/dev/null
    HOLDPID=""
}

# Writes the bitstream file a case uses: the good stand-in, the plain one
# of tools/emu-firmware.sh (which has no sync word: bad), one cut to N
# bytes (cut N), one of N zeros (zeros N), or none (missing).
sla_bitstream() {
    [ "$EMU_DRY" = 0 ] || return 0 # diffcli counting cases: nothing to write
    f=$SIGROK_FIRMWARE_DIR/sysclk-sla5032.bit
    sh "$TOOLS/emu-firmware.sh" "$EMU_FIRMWARE_DIR" >/dev/null || return 1
    case ${1:-good} in
        good | cut) printf '\252\231\125\146' | dd of="$f" bs=1 seek=105 conv=notrunc 2>/dev/null || return 1 ;;
    esac
    case ${1:-good} in
        cut) truncate -s "$2" "$f" ;;
        zeros) head -c "$2" /dev/zero >"$f" ;;
        missing) rm -f "$f" ;;
    esac
}

emu_cases() {
    F=sysclk-sla5032
    # oxrok's log prefix as libsigrok's, and the messages libsigrok's own
    # layers log (usb.c, resource.c) with their prefix; the tool's last line
    # without oxrok's added reason. The filters below read this one form.
    N='s/^oxrok: [a-z]+: oxrok::drivers::sysclk_sla5032(::protocol)?: /sr: sysclk-sla5032: /; s/^sr: sysclk-sla5032: (Could neither|Invalid input)/sr: usb: \1/; s/^sr: sysclk-sla5032: (Failed to open resource)/sr: resource: \1/; s/^Failed to open device: .*$/Failed to open device./; s/invalid argument: .*$/invalid argument./'
    # What oxrok says and the C does not, in one line each, pinned by the
    # "warned" cases: dropped from the others' comparison and nothing else.
    W="/sr: sysclk-sla5032: (The capture ended early|The FPGA's key step failed)/d"
    # The driver's own messages, without the timestamps of -l 5. Dropped:
    # device.c's and hwdriver.c's lines, which are libsigrok's core tagged
    # with the driver's name, not the driver.
    E="$N; /sysclk-sla5032:/!d; s/ *\[[0-9:.]+\]//; /^sr: (device|hwdriver): /d; $W"
    # The driver's and sigrok-cli's errors (a failed open or start). Dropped
    # as well: resource.c's line saying which file it opened at -l 5, and
    # session.c's "Failed to commit", the core on what the driver just said.
    O="$N; /sysclk-sla5032:|Failed|resource/!d; s/ *\[[0-9:.]+\]//; /^sr: (device|hwdriver|session): /d; /^sr: resource: Opened/d; $W"
    # A conn= that selects nothing reads as no conn= at all.
    # The whole output's line count, one line per sample: csv:header=false.
    Q='$=; d'
    # The debug lines of the read-back.
    R="$N; /acquision/!d; s/ *\[[0-9:.]+\]//"
    C='api.c:116-117 reads an empty conn_matches as no conn=, so a conn= that selects nothing lists every SLA5032'
    # Ctrl-C: sigrok-cli has no SIGINT handler and dies (not a driver bug,
    # but the form the harness has for it); oxrok ends the capture. The
    # "lib" cases show what libsigrok's driver does there.
    I='sigrok-cli has no SIGINT handler and dies where oxrok ends the capture'
    # sigrok-cli counts the limit for its decoders and "Device only sent", but
    # writes whole packets to the output (session.c:464), so what the depths
    # round up to comes out. A capture case that is not about this asks for
    # a limit the depths can hold exactly, here 1024 for the same registers
    # as 512 (protocol.c:904-910).
    L='protocol.c:904-910 rounds the depths up, and neither the driver nor sigrok-cli (session.c:464) cuts the output to the limit'
    # What the C does on a chunk that the 2 s timeout ended with data.
    Z='protocol.c:749 checks ret before xfer_len: a chunk with data and a timed out transfer is dropped, exit 0'
    # What the C does on records that claim over 2^31 samples.
    V='protocol.c:782 sums records into an int: it goes negative and g_try_malloc() fails at protocol.c:795-801, so the capture has no samples, exit 0'
    # The C's silence.
    K='protocol.c:646 ignores la_cfg_fpga_done(), and only the debug log tells'
    S='protocol.c:760-765 and 851 take a chunk that is empty or has no marker for the end of the data, and say nothing'
    F2='protocol.c:719-723, 733-737, 749-756 end a capture on an error with only the debug log, exit 0'
    # A capture is held to its first lines and the digest of the whole.
    B='1,4!d'

    sla_bitstream || emu_fail "sysclk-sla5032: could not write the bitstream stand-in"

    # --- The scan and the options, no capture ---
    sla_options
    emu_case "scan" --keep-header -- -d $F --scan
    emu_case "by ids" -- -d $F:conn=2961.66b0 --scan
    emu_case "by address" -- -d $F:conn=$EMU_USB --scan
    # sr_usb_find() matches nothing: no match list, so every SLA5032 is
    # listed (api.c:116-117). oxrok lists what was selected: nothing.
    emu_case "conn matching nothing" --oracle-differs "$C" --no-output -- -d $F:conn=9.99 --scan
    # sr_usb_find() refuses the spec; the scan lists the board all the same.
    emu_case "bad conn" --stderr --filter "$N; /usb:|sysclk-sla5032:/!d; s/ *\[[0-9:.]+\]//" -- -l 4 -d $F:conn=nonsense --scan
    emu_case "bad conn lists" --oracle-differs "$C" --no-output -- -d $F:conn=nonsense --scan
    # Another device's address: the warning of api.c:132-133 (the root hub).
    # Library only: nusb does not list root hubs, so oxrok finds nothing
    # there, and this bus has no other device to name.
    emu_case "lib conn another device" --libsigrok --stderr --filter "$E; s/device [0-9]+\.1 /device BUS.1 /" -- -l 4 -d $F:conn=${EMU_USB%.*}.1 --scan
    # --show opens the device: the bitstream goes up.
    emu_case "show" --keep-header -- -d $F --show
    emu_case "open and close" --stderr --filter "$E" -- -l 5 -d $F --show
    emu_case "get samplerate" -- -d $F --get samplerate
    emu_case "get limit_samples" -- -d $F --get limit_samples
    emu_case "get captureratio" -- -d $F --get captureratio
    emu_case "get rle" -- -d $F --get rle
    emu_case "get unknown refused" --expect-fail --stderr -- -d $F --get voltage_threshold

    # The FPGA keeps its design between sessions: no bitstream, keys kept.
    sla_options configured=1
    emu_case "configured show" -- -d $F --show
    emu_case "configured open" --stderr --no-output --filter "$E" -- -l 5 -d $F --show

    # --- Captures ---
    sla_options configured=1
    # 20 rates, each writing its own divider and multipliers: pll_tbl.
    for r in 500M 400M 250M 200M 100M 50M 25M 20M 10M 5M 2M 1M 500k 200k 100k 50k 20k 10k 5k 2k; do
        emu_case "rate $r" --digest --filter "$B" -- -d $F -C CH0,CH1 -c samplerate=$r --samples 1024 -O bits
    done
    emu_case "rate not listed refused" --expect-fail --stderr --filter "$N" -- -d $F -c samplerate=3M --samples 512 -O bits
    # The driver gets 1024 samples for a limit of 512 at least: both
    # depths have a minimum of 2 * 256 (protocol.c, MAX(pre, 2)). sigrok-cli
    # prints them all; oxrok cuts them, with the same transactions.
    emu_case "limit lowest" --oracle-differs "$L" --digest --filter "$Q" -- -d $F -C CH0,CH3 --samples 512 -O csv:header=false
    emu_case "limit 600" --oracle-differs "$L" --digest --filter "$Q" -- -d $F -C CH0,CH3 --samples 600 -O csv:header=false
    emu_case "limit below the lowest refused" --expect-fail --stderr -- -d $F --samples 511 -O bits
    emu_case "limit above the highest refused" --expect-fail --stderr -- -d $F --samples 67108865 -O bits
    emu_case "samples 5000 hex" --digest --filter "$B" -- -d $F -C CH0,CH1,CH2,CH3 --samples 5000 -O hex
    # The device sends 4608: both tools say so.
    emu_case "samples 5000 falls short" --stderr --filter '/only sent/!d; s/^oxrok: [a-z]+: [^ ]+ //' -- -d $F -C CH0 --samples 5000 -O bits
    emu_case "samples csv" --digest --filter "1,8!d" -- -d $F -C CH4,CH31 --samples 1024 -O csv:header=false
    emu_case "all channels" --digest --filter "1,4!d" -- -d $F --samples 1024 -O bits
    emu_case "ratios" --digest --filter "$B" -- -d $F -C CH0 -c captureratio=10 --samples 4096 -O bits
    for c in 20:2560 30:2000 70:2000 90:5120; do
        ratio=${c%:*}
        emu_case "ratio $ratio" --digest --filter "$B" -- -d $F -C CH0 -c captureratio=$ratio --samples ${c#*:} -O bits
    done
    # Each depth has its minimum, so these two go over any limit.
    for ratio in 0 100; do
        emu_case "ratio $ratio" --oracle-differs "$L" --digest --filter "$Q" -- -d $F -C CH0 -c captureratio=$ratio --samples 2000 -O csv:header=false
    done
    # A ratio the list does not hold is taken all the same (api.c:332).
    emu_case "ratio not listed" --digest --filter "$B" -- -d $F -C CH0 -c captureratio=15 --samples 4096 -O bits
    emu_case "ratio over 100 refused" --expect-fail --stderr --filter "$O" -- -d $F -c captureratio=101 --samples 512 -O bits
    # The time limit becomes samples, in sigrok-cli.
    emu_case "time" --digest --filter "$B" -- -d $F -C CH0 -c samplerate=1M --time 4 -O bits
    emu_case "capture log" --stderr --filter "$E" -- -l 5 -d $F -C CH0 --samples 512 -O bits
    sla_options
    emu_case "capture after a bitstream" --digest --filter "$B" -- -d $F -C CH0,CH1 --samples 1024 -O bits
    sla_options configured=1 wait=5
    emu_case "polls before done" --digest --filter "$B" -- -d $F -C CH0,CH1 --samples 1024 -O bits

    # Long runs: split short of the marker's count (assumed encoder).
    sla_options configured=1 period=200000
    emu_case "runs over a record" --digest --filter "$B" -- -d $F -C CH0 --samples 300000 -O bits

    # All of memory, and over it: pre + post at 64 Mi and 64 Mi + 512
    # (MAX(.., 2) in sla5032_start_acquisition(), against the comment's
    # limit; a device cannot refuse it). To /dev/null: 256 MB of samples.
    sla_options configured=1 period=65536
    emu_case "limit highest" --no-output -- -d $F --samples 67108864 -O binary -o /dev/null
    emu_case "limit highest ratio 0" --no-output -- -d $F -c captureratio=0 --samples 67108864 -O binary -o /dev/null

    # A chunk that fills: 0x100000 records and no marker, then a chunk
    # with only the marker ("no samples", protocol.c).
    sla_options configured=1
    emu_case "chunk exactly full" --digest --filter "$B" -- -d $F -C CH0 --samples 1048576 -O bits
    emu_case "chunk exactly full log" --stderr --filter "$R" -- -l 5 -d $F -C CH0 --samples 1048576 -O bits
    emu_case "two chunks" --digest --filter "$B" -- -d $F -C CH0,CH20 --samples 1500000 -O bits
    # No marker: the second chunk is empty (xfer_len 0).
    sla_options configured=1 no_marker=1
    emu_case "chunk full no marker" --stderr --filter "$R" -- -l 5 -d $F -C CH0 --samples 1048576 -O bits
    emu_case "short chunk no marker" --digest --filter "$B" -- -d $F -C CH0 --samples 1024 -O bits

    # --- Triggers: the registers and the soft trigger over the samples ---
    sla_options configured=1
    emu_case "trigger high" --digest --filter "$B" -- -d $F -C CH0,CH3 -t CH3=1 --samples 1024 -O bits
    emu_case "trigger low" --digest --filter "$B" -- -d $F -C CH0,CH3 -t CH3=0 --samples 1024 -O bits
    emu_case "trigger rising" --digest --filter "$B" -- -d $F -C CH0,CH3 -t CH3=r --samples 1024 -O bits
    emu_case "trigger falling" --digest --filter "$B" -- -d $F -C CH0,CH3 -t CH3=f --samples 1024 -O bits
    emu_case "trigger two channels" --digest --filter "$B" -- -d $F -C CH0,CH3,CH5 -t CH3=r,CH5=1 --samples 2048 -O bits
    emu_case "trigger either edge refused" --expect-fail --stderr -- -d $F -C CH3 -t CH3=e --samples 512 -O bits
    emu_case "trigger marked" --filter '1,8!d' --digest -- -d $F -C CH0,CH3 -t CH3=r -c captureratio=20 --samples 2048 -O csv:header=false:trigger=true
    emu_case "trigger ratio 0" --oracle-differs "$L" --digest --filter "$Q" -- -d $F -C CH0,CH3 -t CH3=r -c captureratio=0 --samples 512 -O csv:header=false
    # On a channel not captured: sigrok-cli refuses it, so api.c:405 and
    # soft-trigger.c's own skip are not reached.
    emu_case "trigger on a disabled channel" --expect-fail --stderr -- -d $F -C CH0 -t CH3=r --samples 512 -O bits
    # Fires in the first chunk, the data goes on in the second.
    emu_case "trigger two chunks" --digest --filter "$B" -- -d $F -C CH0,CH4 -t CH4=r --samples 1500000 -O bits
    # Fires in the second chunk only.
    emu_case "trigger in the second chunk" --digest --filter "$B" -- -d $F -C CH0,CH20 -t CH20=r -c captureratio=90 --samples 1500000 -O bits
    # The top of the counter: it takes 2^31 samples to rise, which the
    # memory does not mind.
    emu_case "trigger late" --digest --filter "$B" -- -d $F -C CH31 -t CH31=r --samples 1024 -O bits
    # CH0 and CH1 never rise together on the counter: the driver has no
    # time limit and polls for ever.
    # oxrok is stopped after the second poll's last reply: open 1, start
    # 2-13, polls 14-16 and 17-19.
    emu_case "trigger never fires" --interrupt in=8:19 --oracle-bug "$I" --no-output -- -d $F -C CH0,CH1 -t CH0=r,CH1=r --samples 512 -O bits
    emu_case "lib trigger never fires" --libsigrok --hangs --timeout 2 --no-output -- -d $F -C CH0,CH1 -t CH0=r,CH1=r --samples 512 -O bits
    # One sequence of the trigger, as sigrok-cli hands it over: two stages.
    emu_case "two trigger stages" --expect-fail --stderr --filter "$O" -- -d $F -C CH1 -t CH1=01 --samples 512 -O bits
    # A client's stop: the C puts nothing more on the wire, at the header or
    # between polls, and neither does oxrok ("trigger never fires").
    emu_case "lib stop at the header" --libsigrok --no-output -- --stop-after 0 -d $F -C CH0 --samples 512 -O bits
    sla_options configured=1 wait=1000
    emu_case "lib stop between polls" --libsigrok --no-output -- --stop-after-ms 150 -d $F -C CH0 --samples 512 -O bits
    sla_options configured=1
    # Recorded for the replay tests: the board has its design, since the
    # replay's stand-in bitstream has no sync word. The fixtures hold the
    # records as hex, about 12 KB each.
    emu_case "recorded capture" --record --digest -- -d $F -C CH0,CH1 --samples 1024 -O bits
    emu_case "recorded trigger" --record --digest -- -d $F -C CH0,CH3 -t CH3=r --samples 1024 -O bits

    # --- Faults of the open ---
    # An open: read-reg 1 (is it configured?), read-reg 2 (ignored), init,
    # chunks, read-mem 1, write-reg 1-2 (keys), read-reg 3 (ignored),
    # write-reg 3 (R14, ignored).
    for f in stall=read-reg:1 stall_reply=read-reg:1 short_reply=read-reg:1 stall=init stall=chunk:1 stall=chunk:13; do
        sla_options $f
        emu_case "fault open $f" --expect-fail --stderr --filter "$O" -- -l 5 -d $F --show
    done
    # read-reg 2 and 3 and write-reg 3 are ignored: the capture still works.
    for f in stall=read-reg:2 stall_reply=read-reg:3 short_reply=read-reg:3 stall=write-reg:3; do
        sla_options $f
        emu_case "fault open ignored $f" --digest --filter "$B" -- -d $F -C CH0 --samples 1024 -O bits
    done
    # The key's failures are ignored too (protocol.c:646, la_cfg_fpga_done()'s
    # result): the open still succeeds. These assume nothing about the board.
    for f in stall=read-mem stall=write-reg:1; do
        sla_options $f
        emu_case "fault keys ignored $f" --stderr --filter "$E" -- -l 5 -d $F --show
    done
    # The open stays good on oxrok as well, which says what went wrong.
    sla_options stall=read-mem
    emu_case "fault keys warned" --oracle-differs "$K" --stderr --filter "$N; /key step failed/!d" -- -d $F --show
    # Assumed, not the C's: an FPGA without its keys never samples, so
    # register 2 repeats A5A5A5A0 and the capture never ends. oxrok is
    # stopped after the second poll's last reply. Replies: 2 of the open, the
    # key's if the read reached a reply, 1 of register 2 after the keys never
    # (each of these stops the key step), 12 of the start, 3 per poll.
    for c in stall=read-mem:20 stall_reply=read-mem:21 short_reply=read-mem:21 stall=write-reg:1:21 stall=write-reg:2:21; do
        kf=${c%:*}
        sla_options $kf
        emu_case "fault keys $kf" --interrupt in=8:${c##*:} --oracle-bug "$I" --no-output -- -d $F -C CH0 --samples 512 -O bits
        emu_case "lib fault keys $kf" --libsigrok --hangs --timeout 2 --no-output -- -d $F -C CH0 --samples 512 -O bits
    done
    # Neither tool gives the first reply up before 500 s, and an open has
    # nothing to stop: the signal ends oxrok there as it ends any program.
    sla_options hold_reply=read-reg:1
    emu_case "fault no answer at the open" --interrupt out=4:1 --oracle-bug "$I" --expect-fail --no-output -- -d $F -C CH0 --samples 512 -O bits
    emu_case "lib fault no answer at the open" --libsigrok --hangs --timeout 2 --no-output -- -d $F -C CH0 --samples 512 -O bits
    # emu_case begins with a reset, which re-attaches only when the
    # descriptors change: they never do here.
    sla_options
    if sla_hold; then
        emu_case "interface claimed by another program" --expect-fail --stderr --filter "$O" -- -d $F --show
        sla_release
    fi

    # The whole upload ends on a chunk: no short last one.
    sla_bitstream cut 2849 || emu_fail "sysclk-sla5032: could not cut the bitstream stand-in"
    sla_options fw_size=2849
    emu_case "bitstream of whole chunks" -- -d $F --show
    # Broken files: the plain stand-in has no sync word.
    sla_options
    sla_bitstream bad || emu_fail "sysclk-sla5032: could not write the plain stand-in"
    emu_case "bitstream file without sync word" --expect-fail --stderr --filter "$O" -- -d $F --show
    sla_bitstream zeros 105 || emu_fail "sysclk-sla5032: could not write the short file"
    emu_case "bitstream file too small" --expect-fail --stderr --filter "$O" -- -d $F --show
    sla_bitstream zeros 524289 || emu_fail "sysclk-sla5032: could not write the huge file"
    emu_case "bitstream file too large" --expect-fail --stderr --filter "$O" -- -d $F --show
    sla_bitstream missing || emu_fail "sysclk-sla5032: could not move the file"
    emu_case "bitstream file missing" --expect-fail --stderr --filter "$O" -- -d $F --show
    sla_bitstream || emu_fail "sysclk-sla5032: could not write the bitstream stand-in back"

    # --- Faults of a capture's start; each one fails it ---
    # With configured=1 the open is read-reg 1. Then write-reg 1-2 (depth),
    # 3-5 (trigger), 6 (divider), read-reg 2 and write-reg 7 (bit 0),
    # read-reg 3 and write-reg 8 (bit 7), 9-12 (PWM), 13 (R14), then ten
    # read-modify-writes of R5: read-reg 4-13, write-reg 14-23.
    for f in 1 2 3 4 5 6 7 8 9 10 11 12 13 14 23; do
        sla_options configured=1 stall=write-reg:$f
        emu_case "fault start write-reg $f" --expect-fail --stderr --filter "$O" -- -d $F -C CH0 --samples 512 -O bits
    done
    for f in 2 3 4 13; do
        sla_options configured=1 stall=read-reg:$f
        emu_case "fault start read-reg $f" --expect-fail --stderr --filter "$O" -- -d $F -C CH0 --samples 512 -O bits
    done

    # --- Faults of the wait and the read-back; each one ends it, exit 0 ---
    # After the start: read-reg 14-16 (R1, R0, R2 of the first poll),
    # 17-19, 20-22 (done); write-reg 24-25 (0x08, 0x28), read-data 1,
    # write-reg 26 (R3), read-reg 23, write-reg 27, read-reg 24, write-reg
    # 28 (bit 4), the chunk; write-reg 29 (R14).
    for f in stall=read-reg:14 stall=read-reg:15 stall=read-reg:16 stall_reply=read-reg:14 short_reply=read-reg:16 stall=write-reg:24 stall=write-reg:25 stall=read-data stall=write-reg:26 stall=read-reg:23 stall=write-reg:27 stall=read-reg:24 stall=write-reg:28; do
        sla_options configured=1 $f
        emu_case "fault capture $f" --oxrok-fails "command 3 failed: LIBUSB_ERROR_PIPE or command 4 failed: LIBUSB_ERROR_PIPE ..." --stderr --filter "$E" -- -l 5 -d $F -C CH0 --samples 512 -O bits
    done
    # What only the debug log says in the C.
    sla_options configured=1 stall=read-reg:14
    emu_case "fault capture warned" --oxrok-fails "command 3 failed: LIBUSB_ERROR_PIPE" --oracle-differs "$F2" --stderr --filter "$N; /The capture ended early/!d" -- -d $F -C CH0 --samples 512 -O bits
    sla_options configured=1 data_stall=1
    emu_case "fault chunk stalled" --oxrok-fails "endpoint stalled" --stderr --filter "$E" -- -l 5 -d $F -C CH0 --samples 512 -O bits
    # The driver gives up after USB_DATA_TIMEOUT_MS, 2 s.
    sla_options configured=1 data_stall=2
    emu_case "fault second chunk stalled" --oxrok-fails "endpoint stalled" --stderr --filter "$E" -- -l 5 -d $F -C CH0 --samples 1500000 -O bits
    sla_options configured=1 data_hold=1
    emu_case "fault chunk never sent" --oxrok-fails "timeout" --stderr --filter "$E" -- -l 5 -d $F -C CH0 --samples 512 -O bits
    # The records end on a packet boundary and no zero-length packet comes:
    # 65536 samples at 258 a run are 255 records and the marker, 1536 bytes.
    # The transfer runs to the 2 s timeout with the whole capture in it.
    sla_options configured=1 period=258 no_zlp=1
    emu_case "fault chunk with no zlp" --oracle-differs "$Z" --digest --filter "$B" -- -d $F -C CH0 --samples 65536 -O bits
    emu_case "fault chunk with no zlp log" --oracle-differs "$Z" --stderr --filter "$E" -- -l 5 -d $F -C CH0 --samples 65536 -O bits
    # What the C ends in silence: nothing at all (protocol.c:760-765), and a
    # chunk neither full nor with a marker (protocol.c:851).
    sla_options configured=1 data_short=0
    emu_case "fault chunk empty warned" --oxrok-fails "the device sent no samples" --oracle-differs "$S" --stderr --filter "$N; /The capture ended early/!d" -- -d $F -C CH0 --samples 512 -O bits
    sla_options configured=1 data_short=600
    emu_case "fault chunk short warned" --oxrok-fails "a chunk of 600 bytes has no end marker" --oracle-differs "$S" --stderr --filter "$N; /The capture ended early/!d" -- -d $F -C CH0 --samples 512 -O bits
    sla_options configured=1 data_short=0
    emu_case "fault chunk empty" --oxrok-fails "the device sent no samples" --stderr --filter "$E" -- -l 5 -d $F -C CH0 --samples 512 -O bits
    sla_options configured=1 data_short=601
    emu_case "fault chunk short and uneven" --oxrok-fails "a chunk of 601 bytes has no end marker" --stderr --filter "$E" -- -l 5 -d $F -C CH0 --samples 512 -O bits
    sla_options configured=1 data_short=600
    emu_case "fault chunk short samples" --oxrok-fails "a chunk of 600 bytes has no end marker" --digest --filter "$B" -- -d $F -C CH0 --samples 512 -O bits
    # Records claiming over 2^31 samples: the int at protocol.c:782 goes
    # negative and g_try_malloc() fails (protocol.c:797-801). About 197 KB.
    # oxrok counts in 64 bits and lets the limit's samples out, expanding no
    # more than those.
    sla_options configured=1 overcount=1
    emu_case "fault samples over an int" --oracle-differs "$V" --stderr --filter "$E" -- -l 5 -d $F -C CH0 --samples 512 -O bits
    emu_case "fault samples over an int cut" --oracle-differs "$V" --digest --filter "$B" -- -d $F -C CH0 --samples 512 -O bits
    # No answer to a poll: the driver waits 500 s (USB_REPLY_TIMEOUT_MS).
    sla_options configured=1 hold_reply=read-reg:16
    emu_case "lib fault no answer to a poll" --libsigrok --hangs --timeout 2 --no-output -- -d $F -C CH0 --samples 512 -O bits
    # oxrok is stopped at the reply before: the wait ends in its next slice,
    # with no command on top of the reply unread.
    emu_case "fault no answer to a poll" --interrupt in=8:15 --oracle-bug "$I" --no-output -- -d $F -C CH0 --samples 512 -O bits
    # Unplugged at the first poll's status reply.
    sla_options configured=1
    emu_case "unplug while polling" --oxrok-fails "command 3 failed: LIBUSB_ERROR_NO_DEVICE" --unplug in=8:16 --stderr --filter "$E" -- -l 5 -d $F -C CH0 --samples 512 -O bits
    # Unplugged at bit 4's rise, with the chunk still to read (the 53rd
    # command: 36 of the start, 9 of the polls, 8 of the read-back).
    emu_case "unplug at the chunk" --oxrok-fails "device disconnected" --unplug out=4:53 --stderr --filter "$E" -- -l 5 -d $F -C CH0 --samples 512 -O bits
}
