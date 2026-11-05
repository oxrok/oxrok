# A PCE PCE-322A (emu/src/models/pce-322a.rs) on the CDC-ACM shell.
# Sourced by tools/diffcli.sh --emu; see docs/USB.md, "Writing a serial model".
EMU_MODEL=pce-322a
EMU_CDC=1
EMU_DRIVER=pce-322a
EMU_GCOV=pce-322a
EMU_GCOV_SHARED="src/serial.c src/serial_libsp.c"

emu_cases() {
    emu_conns
    # A frame on the stream, as the model's defaults make it.
    G=7f028e00000026110512300000
    # Most of what follows has no use for the stream, and frames=0 keeps it from
    # the .tx whatever the timing.
    emu_options frames=0
    emu_case "scan" --keep-header -- -d $A --scan
    emu_case "show" --keep-header -- -d $A --show
    emu_case "no such node" --keep-header --no-output -- -d pce-322a:conn=/dev/ttyACM97 --scan
    emu_case "no conn" --keep-header --no-output -- -d pce-322a --scan
    emu_case "show without a device" --keep-header --stderr --expect-fail -- -d pce-322a --show
    emu_case "serialcomm is not a scan option" --keep-header --stderr -- -d $A:serialcomm=9600/8n1 --scan
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
    emu_case "set freq Z" --stderr --expect-fail -- -d $A -c spl_weight_freq=Z --set
    emu_case "set time S" --no-output -- -d $A -c spl_weight_time=S --set
    emu_case "set time F" --no-output -- -d $A -c spl_weight_time=F --set
    emu_case "set time X" --stderr --expect-fail -- -d $A -c spl_weight_time=X --set
    emu_case "set range 30-130" --no-output -- -d $A -c spl_meas_range=30-130 --set
    emu_case "set range 30-80" --no-output -- -d $A -c spl_meas_range=30-80 --set
    emu_case "set range 50-100" --no-output -- -d $A -c spl_meas_range=50-100 --set
    emu_case "set range 80-130" --no-output -- -d $A -c spl_meas_range=80-130 --set
    emu_case "set range 30-100" --stderr --expect-fail -- -d $A -c spl_meas_range=30-100 --set
    emu_case "set range no number" --stderr --expect-fail -- -d $A -c spl_meas_range=loud --set
    emu_case "set power_off" --no-output -- -d $A -c power_off=true --set
    emu_case "set power_off false" --no-output -- -d $A -c power_off=false --set
    emu_case "set data_source" -- -d $A -c data_source=Memory --get data_source
    emu_case "set data_source Tape" --stderr --expect-fail -- -d $A -c data_source=Tape --set
    emu_case "set continuous" --stderr --expect-fail -- -d $A -c continuous=true --set
    emu_case "set limit_samples" -- -d $A -c limit_samples=7 --get limit_samples
    emu_case "time" --stderr --expect-fail -- -d $A --time 100
    emu_case "frames" --stderr --expect-fail -- -d $A --frames 1
    # A setting's reply never changes what C says it holds, until a frame comes.
    emu_case "set freq C then get" -- -d $A -c spl_weight_freq=C --get spl_weight_freq
    emu_case "set time S then get" -- -d $A -c spl_weight_time=S --get spl_weight_time
    emu_case "set range then get" -- -d $A -c spl_meas_range=50-100 --get spl_meas_range
    # Set twice before a frame: C counts both from what the last frame said.
    emu_case "set freq twice" --no-output -- -d $A -c spl_weight_freq=C:spl_weight_freq=C --set
    emu_case "set range twice" --no-output -- -d $A -c spl_meas_range=30-80:spl_meas_range=50-100 --set

    # The stream.
    emu_options frames=1
    emu_case "one sample" -- -d $A --samples 1
    emu_case "one sample csv" -- -d $A --samples 1 -O csv:header=false:time=false
    emu_case "one sample analog" -- -d $A --samples 1 -O analog
    emu_case "rename" -- -d $A -C SPL=Noise --samples 1
    emu_case "no such channel" --stderr --expect-fail -- -d $A -C Noise --samples 1
    emu_options frames=3 period=5
    emu_case "three samples" -- -d $A --samples 3
    emu_case "config limit_samples" -- -d $A -c limit_samples=3 --samples 3
    emu_case "lib limit_samples" --libsigrok -- -d $A -c limit_samples=2 --continuous
    emu_case "lib continuous" --libsigrok --no-output -- -d $A --continuous --stop-after 0
    emu_case "lib stop after one" --libsigrok -- -d $A --continuous --stop-after 1
    emu_options frames=1 chunk=1 period=1000
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
    emu_case "settings then a sample" -- -d $A -c spl_weight_freq=C:spl_weight_time=S:spl_meas_range=80-130 --samples 1
    emu_options frames=1 period=100 freq=c time=s
    emu_case "settings the frame undoes" -- -d $A -c spl_weight_freq=A:spl_weight_time=F --samples 1
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
    M="-c data_source=Memory"
    emu_options frames=0
    emu_case "memory" --stderr -- -d $A $M --samples 1000
    emu_case "memory stdout" -- -d $A $M --samples 1000
    emu_case "memory cut" -- -d $A $M --samples 3
    emu_case "lib memory" --libsigrok -- -d $A $M --continuous
    emu_case "lib stop in memory" --libsigrok --no-output -- -d $A $M --continuous --stop-after 0
    emu_options frames=0 records=0
    emu_case "memory empty" --stderr -- -d $A $M --samples 1000
    # 611 bytes.
    emu_options frames=0 records=1 samples=300
    emu_case "memory three blocks" --stderr -- -d $A $M --samples 1000
    emu_case "memory three blocks stdout" --digest -- -d $A $M --samples 1000
    # 255 bytes: the last block's length at its most.
    emu_options frames=0 records=1 samples=122
    emu_case "memory last at 255" --stderr -- -d $A $M --samples 1000
    # A whole block and an empty last one.
    emu_options frames=0 records=1 samples=122 used=1 last=0
    emu_case "memory whole block" -- -d $A $M --samples 1000
    # A header split between two blocks: 125 bytes each.
    emu_options frames=0 records=3 samples=57
    emu_case "memory a header across blocks" --stderr -- -d $A $M --samples 1000
    # Past what is stored the blocks are ff, which reads as a header that fails.
    emu_options frames=0 records=1 samples=2 used=1 last=2
    emu_case "memory past the stored" -- -d $A $M --samples 1000
    # The wiki's other header, dB(C) by its flags, and any other flags as A.
    emu_options frames=0 memory=ffff01000c2611051200308e02
    emu_case "memory header ffff" --stderr -- -d $A $M --samples 1000
    emu_options frames=0 memory=7ff701000b261105d200308e02
    emu_case "memory odd flags and hour" --stderr -- -d $A $M --samples 1000
    # Header bytes that fail on what comes next.
    emu_options frames=0 memory=7ff702000a2611051200308e02
    emu_case "memory not a header" --stderr -- -d $A $M --samples 1000
    # Samples with no header, and one that looks like a header's start.
    emu_options frames=0 memory=8e028f02
    emu_case "memory no header" --stderr -- -d $A $M --samples 1000
    emu_options frames=0 memory=7ff701000a2611051200307ff78e0201000a261105120031
    emu_case "memory a sample like a header" --stderr -- -d $A $M --samples 1000
    # What the status's window does: stale bytes first, and one that isn't it.
    emu_options frames=0 status=${G}d1050001d200003620
    emu_case "memory stale before the status" -- -d $A $M --samples 1000
    emu_options frames=0 status=d1050001d200003621d1050001d200003620
    emu_case "memory a wrong status" -- -d $A $M --samples 1000
    # C keeps reading what it asked for: no answer, no end.
    emu_options frames=0 no-status=1
    emu_case "memory no status" --hangs --timeout 2 --no-output -- -d $A $M --samples 1000
    emu_options frames=0 status=d1050001d200003610
    emu_case "memory status ending in 10" --hangs --timeout 2 --no-output -- -d $A $M --samples 1000
    emu_options frames=0 records=1 samples=300 no-block=1
    emu_case "memory a block lost" --hangs --timeout 2 -- -d $A $M --samples 1000

    # What ends no capture in C.
    emu_options frames=0
    emu_case "silent" --hangs --timeout 2 --no-output -- -d $A --samples 1
    emu_options frames=1 period=5
    emu_case "power off" --hangs --timeout 2 --no-output -- -d $A -c power_off=true --samples 1
    emu_options frames=1 uart=9600/8n1
    emu_case "another line" --hangs --timeout 2 --no-output -- -d $A --samples 1

    # The USB side.
    emu_options frames=0 stall=line
    emu_case "line coding stalled" --keep-header -- -d $A --scan
    emu_options frames=0 stall=lines
    emu_case "control lines stalled" --keep-header -- -d $A --scan
    emu_options frames=2 period=5
    emu_case "unplug" --hangs --unplug model --timeout 20 -- -d $A --samples 3
    emu_options frames=0 no-status=1
    emu_case "unplug in memory" --hangs --unplug model --timeout 8 --no-output -- -d $A $M --samples 1000
    # The meter leaves as the close's disconnect goes out: C drops the failure.
    emu_options frames=0
    emu_case "unplug at the close" --unplug out=2:2 --no-output -- -d $A -c spl_weight_freq=C --set
    emu_options
}

# See emu_options in tools/diffcli.sh.
emu_conns() {
    A=pce-322a:conn=$EMU_TTY
}
