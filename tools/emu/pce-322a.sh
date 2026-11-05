# A PCE PCE-322A (emu/src/models/pce-322a.rs) on the CDC-ACM shell.
# Sourced by tools/diffcli.sh --emu; see docs/USB.md, "Writing a serial model".
EMU_MODEL=pce-322a
EMU_CDC=1
EMU_DRIVER=pce-322a
EMU_FULL=$EMU_DRIVER
EMU_GCOV=pce-322a
EMU_GCOV_SHARED="src/serial.c src/serial_libsp.c"

emu_cases() {
    emu_conns
    # A frame on the stream, as the model's defaults make it.
    G=7f028e00000026110512300000
    # oxrok's log prefix, as C's.
    F='s/^oxrok: [a-z]+: oxrok::drivers::pce_322a(::protocol)?: /sr: pce-322a: /'
    # What sigrok-cli's refusal skips and oxrok's does not: the close's disconnect,
    # which ends the one line of what went out. "set freq C" and the like pin it.
    P='s/^(bulk_out 02 [0-9a-f]*)caff -> [0-9]+$/\1/
s/^(bulk_out 02 [0-9a-f]+) -> [0-9]+$/\1/'
    # What oxrok adds after the error sigrok-cli names.
    FN="$F
s/^(Failed to set device option .[a-z_]+.: invalid argument).*$/\\1./"
    SET_BUG="C takes what the meter holds from readings alone (protocol.c:374,395,452), so a set leaves it as it was: a get says the old value, and the next set counts from there; oxrok counts a sent toggle as done"
    CUT_BUG="C sends all of a memory capture past its limit, by choice (protocol.c:92), and sigrok-cli shows it; oxrok honours the limit in the output, with the same requests"
    HANG_BUG="with no answer C waits for ever, as no read has a bound (protocol.c:285,312,321) and no limit counts what never came; oxrok ends after 5 s, and that wait is what this case costs"
    TEN_BUG="the wiki gives a status's last byte as 10 or 20, and protocol.c:185 takes 20 alone, so C waits for ever; oxrok reads the memory"
    GONE_BUG="on a port that is gone the source's revents is no longer G_IO_IN alone, so protocol.c:285,312,321 skip the read and the callback returns TRUE for ever: this stays in the loop at 20 s and on. oxrok's read fails and ends the capture"
    SIGINT_BUG="sigrok-cli has no SIGINT handler, it dies where oxrok ends the capture"
    # Most of what follows has no use for the stream, and frames=0 keeps it from
    # the .tx whatever the timing.
    emu_options frames=0
    emu_case "scan" --keep-header -- -d $A --scan
    emu_case "show" --keep-header -- -d $A --show
    emu_case "no such node" --keep-header --no-output -- -d pce-322a:conn=/dev/ttyACM97 --scan
    emu_case "no conn" --keep-header --no-output -- -d pce-322a --scan
    emu_case "show without a device" --keep-header --stderr --filter "$F" --expect-fail -- -d pce-322a --show
    # What a scan does on a meter already streaming: it reads nothing, so what
    # comes is the kernel's to hold, and how much of it depends on the time.
    emu_options unprompted=1 period=2
    emu_case "scan a streaming meter" --keep-header --tx-filter '/^bulk_in 82 /d' -- -d $A --scan
    # Settings, one per path.
    emu_options frames=0
    emu_case "get limit_samples" -- -d $A --get limit_samples
    emu_case "get spl_weight_freq" -- -d $A --get spl_weight_freq
    emu_case "get spl_weight_time" -- -d $A --get spl_weight_time
    emu_case "get spl_meas_range" -- -d $A --get spl_meas_range
    emu_case "get power_off" -- -d $A --get power_off
    emu_case "get data_source" -- -d $A --get data_source
    emu_case "set freq C" --no-output -- -d $A -c spl_weight_freq=C --set
    emu_case "set freq A" --no-output -- -d $A -c spl_weight_freq=A --set
    emu_case "set freq Z" --stderr --filter "$FN" --tx-filter "$P" --expect-fail -- -d $A -c spl_weight_freq=Z --set
    emu_case "set time S" --no-output -- -d $A -c spl_weight_time=S --set
    emu_case "set time F" --no-output -- -d $A -c spl_weight_time=F --set
    emu_case "set time X" --stderr --filter "$FN" --tx-filter "$P" --expect-fail -- -d $A -c spl_weight_time=X --set
    emu_case "set range 30-130" --no-output -- -d $A -c spl_meas_range=30-130 --set
    emu_case "set range 30-80" --no-output -- -d $A -c spl_meas_range=30-80 --set
    emu_case "set range 50-100" --no-output -- -d $A -c spl_meas_range=50-100 --set
    emu_case "set range 80-130" --no-output -- -d $A -c spl_meas_range=80-130 --set
    emu_case "set range 30-100" --stderr --filter "$FN" --tx-filter "$P" --expect-fail -- -d $A -c spl_meas_range=30-100 --set
    emu_case "set power_off" --no-output -- -d $A -c power_off=true --set
    emu_case "set power_off false" --no-output -- -d $A -c power_off=false --set
    emu_case "set data_source" -- -d $A -c data_source=Memory --get data_source
    emu_case "set data_source Tape" --stderr --filter "$FN" --tx-filter "$P" --expect-fail -- -d $A -c data_source=Tape --set
    emu_case "set continuous" --stderr --filter "$F" --tx-filter "$P" --expect-fail -- -d $A -c continuous=true --set
    emu_case "set limit_samples" -- -d $A -c limit_samples=7 --get limit_samples
    emu_case "time" --stderr --filter "$F" --tx-filter "$P" --expect-fail -- -d $A --time 100
    emu_case "frames" --stderr --filter "$F" --tx-filter "$P" --expect-fail -- -d $A --frames 1
    # A setting never changes what C says the meter holds, until a frame comes.
    emu_case "set freq C then get" --oracle-differs "$SET_BUG" -- -d $A -c spl_weight_freq=C --get spl_weight_freq
    emu_case "set time S then get" --oracle-differs "$SET_BUG" -- -d $A -c spl_weight_time=S --get spl_weight_time
    emu_case "set range then get" --oracle-differs "$SET_BUG" -- -d $A -c spl_meas_range=50-100 --get spl_meas_range
    # Set twice before a frame: C counts both from what the last frame said, so
    # it toggles the weighting back and steps the range one too far. In one -c the
    # last alone counts, in both tools.
    emu_case "set in one -c" --no-output -- -d $A -c spl_meas_range=30-80:spl_meas_range=50-100 --set
    emu_case "set freq twice" --oracle-differs "$SET_BUG" --no-output -- -d $A -c spl_weight_freq=C -c spl_weight_freq=C --set
    emu_case "set range twice" --oracle-differs "$SET_BUG" --no-output -- -d $A -c spl_meas_range=30-80 -c spl_meas_range=50-100 --set

    # The stream.
    emu_options frames=1
    emu_case "one sample" -- -d $A --samples 1
    emu_case "one sample csv" -- -d $A --samples 1 -O csv:header=false:time=false
    emu_case "one sample analog" -- -d $A --samples 1 -O analog
    emu_case "rename" -- -d $A -C SPL=Noise --samples 1
    emu_options frames=3 period=5
    emu_case "three samples" -- -d $A --samples 3
    emu_case "config limit_samples" -- -d $A -c limit_samples=3 --samples 3
    emu_case "lib limit_samples" --libsigrok --oxrok -- -d $A -c limit_samples=2 --continuous
    emu_case "lib continuous" --libsigrok --no-output -- -d $A --continuous --stop-after 0
    emu_case "lib stop after one" --libsigrok -- -d $A --continuous --stop-after 1
    emu_options frames=1 chunk=1
    emu_case "one byte per transfer" -- -d $A --samples 1
    # How the meter is set, in what comes.
    emu_options frames=1 freq=c time=s mode=max range=2 spl=1000
    emu_case "flags" -- -d $A --samples 1
    emu_options frames=1 mode=min
    emu_case "min" -- -d $A --samples 1
    # Max goes first when both are set, and each frame stands alone.
    emu_options period=5 frames=4 replies=7f028e00000426110512300000,7f028e000c0026110512300000,$G,7f028e01080026110512300000
    emu_case "flags frame by frame" -- -d $A --samples 4
    # The value's two ends, and the high bits of the flags' bytes.
    emu_options period=5 frames=2 replies=7f000000000026110512300000,7fffff00f00026110512300000
    emu_case "the values' ends" -- -d $A --samples 2
    # Settings, with a long period: a frame must not beat the second write.
    emu_options frames=1 period=100
    emu_case "settings then a sample" -- -d $A -c spl_weight_freq=C -c spl_weight_time=S -c spl_meas_range=80-130 --samples 1
    emu_options frames=1 period=100 freq=c time=s
    emu_case "settings the frame undoes" -- -d $A -c spl_weight_freq=A -c spl_weight_time=F --samples 1
    emu_options frames=1 period=100 freq=c
    emu_case "toggles a meter at C" -- -d $A -c spl_weight_freq=C --samples 1
    emu_options frames=1 period=100 range=3
    emu_case "range on a meter elsewhere" -- -d $A -c spl_meas_range=30-80 --samples 1
    # What the window does with what isn't a frame.
    emu_options period=5 frames=2 replies=55aa,$G
    emu_case "stray bytes first" -- -d $A --samples 1
    emu_options period=5 frames=2 replies=7f028e00000026110512300001,$G
    emu_case "a frame with no end" -- -d $A --samples 1
    # An early 7f takes the frame's place, with all its bytes one along.
    emu_options frames=1 period=5 replies=7f7f028e000000261105123000
    emu_case "a stray 7f" -- -d $A --samples 1
    # Thirteen bytes that pass for a frame, straddling what's real: 7f at 2,
    # 00 at 14.
    emu_options period=5 frames=2 replies=55557f111100000000000000000000,$G
    emu_case "a frame by accident" -- -d $A --samples 2

    # Memory.
    # Its words go through the library runner: a limit not reached has oxrok's CLI say
    # how many samples came, which C does not, and memory has no use for a limit.
    M="-c data_source=Memory"
    emu_options frames=0
    emu_case "memory" --libsigrok --oxrok --stderr --filter "$F" -- -d $A $M --continuous
    emu_case "memory stdout" -- -d $A $M --samples 1000
    emu_case "memory cut" --oracle-differs "$CUT_BUG" -- -d $A $M --samples 3
    emu_case "lib memory" --libsigrok --oxrok -- -d $A $M --continuous
    emu_case "lib stop in memory" --libsigrok --no-output -- -d $A $M --continuous --stop-after 0
    emu_options frames=0 records=0
    emu_case "memory empty" --libsigrok --oxrok --stderr --filter "$F" -- -d $A $M --continuous
    # 611 bytes.
    emu_options frames=0 records=1 samples=300
    emu_case "memory three blocks" --libsigrok --oxrok --stderr --filter "$F" -- -d $A $M --continuous
    emu_case "memory three blocks stdout" --digest -- -d $A $M --samples 1000
    # 255 bytes: the last block's length at its most.
    emu_options frames=0 records=1 samples=122
    emu_case "memory last at 255" --libsigrok --oxrok --stderr --filter "$F" -- -d $A $M --continuous
    # A whole block and an empty last one.
    emu_options frames=0 records=1 samples=122 used=1 last=0
    emu_case "memory whole block" -- -d $A $M --samples 1000
    # A header split between two blocks: 125 bytes each.
    emu_options frames=0 records=3 samples=57
    emu_case "memory a header across blocks" --libsigrok --oxrok --stderr --filter "$F" -- -d $A $M --continuous
    # Past what is stored the blocks are ff, which reads as a header that fails.
    emu_options frames=0 records=1 samples=2 used=1 last=2
    emu_case "memory past the stored" -- -d $A $M --samples 1000
    # The wiki's other header, dB(C) by its flags, and any other flags as A.
    emu_options frames=0 memory=ffff01000c2611051200308e02
    emu_case "memory header ffff" --libsigrok --oxrok --stderr --filter "$F" -- -d $A $M --continuous
    emu_options frames=0 memory=7ff701000b261105d200308e02
    emu_case "memory odd flags and hour" --libsigrok --oxrok --stderr --filter "$F" -- -d $A $M --continuous
    # Header bytes that fail on what comes next.
    emu_options frames=0 memory=7ff702000a2611051200308e02
    emu_case "memory not a header" --libsigrok --oxrok --stderr --filter "$F" -- -d $A $M --continuous
    # Samples with no header, and one that looks like a header's start.
    emu_options frames=0 memory=8e028f02
    emu_case "memory no header" --libsigrok --oxrok --stderr --filter "$F" -- -d $A $M --continuous
    emu_options frames=0 memory=7ff701000a2611051200307ff78e0201000a261105120031
    emu_case "memory a sample like a header" --libsigrok --oxrok --stderr --filter "$F" -- -d $A $M --continuous
    # What the status's window does: stale bytes first, and one that isn't it.
    emu_options frames=0 status=${G}d1050001d200003620
    emu_case "memory stale before the status" -- -d $A $M --samples 1000
    emu_options frames=0 status=d1050001d200003621d1050001d200003620
    emu_case "memory a wrong status" -- -d $A $M --samples 1000
    # No answer: C goes on waiting, and oxrok ends at its bound. 7 s each, for C's
    # wait and oxrok's 5 s, one per step of a read.
    emu_options frames=0 no-status=1
    emu_case "memory no status" --oracle-bug "$HANG_BUG" --oracle-hangs --timeout 7 --no-output -- -d $A $M --samples 1000
    emu_options frames=0 status=d1050001d200003610
    emu_case "memory status ending in 10" --oracle-bug "$TEN_BUG" --oracle-hangs --timeout 3 -- -d $A $M --samples 1000
    emu_options frames=0 records=1 samples=300 no-block=1
    emu_case "memory a block lost" --oracle-bug "$HANG_BUG" --oracle-hangs --timeout 7 -- -d $A $M --samples 1000

    # What ends no capture in C.
    emu_options frames=0
    emu_case "silent" --oracle-bug "$HANG_BUG" --oracle-hangs --timeout 7 --no-output -- -d $A --samples 1
    # The same way out for oxrok, so C's alone.
    emu_options frames=1 period=5
    emu_case "lib power off" --libsigrok --hangs --timeout 2 --no-output -- -d $A -c power_off=true --samples 1
    emu_options frames=1 uart=9600/8n1
    emu_case "lib another line" --libsigrok --hangs --timeout 2 --no-output -- -d $A --samples 1

    # The USB side.
    emu_options frames=0 stall=line
    emu_case "line coding stalled" --keep-header -- -d $A --scan
    emu_options frames=0 stall=lines
    emu_case "control lines stalled" --keep-header -- -d $A --scan
    emu_options frames=2 period=5
    emu_case "unplug" --oracle-bug "$GONE_BUG" --oracle-hangs --unplug model --timeout 5 -- -d $A --samples 3
    emu_options frames=0 no-status=1
    emu_case "unplug in memory" --oracle-bug "$GONE_BUG" --oracle-hangs --unplug model --timeout 5 --no-output -- -d $A $M --samples 1000

    # What oxrok alone can run. The model has gone quiet: the limit is not met,
    # and each of these ends well before the bound would.
    emu_options frames=2 period=5
    emu_case "interrupt" --oracle-bug "$SIGINT_BUG" --interrupt model --timeout 3 -- -d $A --samples 3
    emu_options frames=0 records=1 samples=300 no-block=1
    emu_case "interrupt in memory" --oracle-bug "$SIGINT_BUG" --interrupt model --timeout 3 -- -d $A $M --samples 1000
    emu_options
}

# See emu_options in tools/diffcli.sh.
emu_conns() {
    A=pce-322a:conn=$EMU_TTY
}
