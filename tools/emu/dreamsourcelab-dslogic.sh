# The DreamSourceLab DSLogic family (emu/src/models/dreamsourcelab-dslogic.rs):
# an FX2 without firmware that takes the upload and renumerates, then
# an FPGA that takes its bitstream and a setting packet per capture, as
# the DSLogic Plus (the default), Basic, Pro, the first DSLogic and the
# DSCope, and the same with the firmware already running. Sourced by
# tools/diffcli.sh --emu; see docs/USB.md, "Writing a device model".
EMU_MODEL=dreamsourcelab-dslogic
# The session attaches the board with its firmware running, so that the
# first case can name it by the address of that attach (EMU_USB): a
# board without firmware renumerates under the first tool and is
# attached again, at another address, for the second.
EMU_OPTIONS="-o profile=plus -o flashed=1"
EMU_DRIVER=dreamsourcelab-dslogic
EMU_GCOV_SHARED="src/usb.c src/ezusb.c"
# oxrok has the driver: its cases must run full.
EMU_FULL=dreamsourcelab-dslogic

# The model's options for the cases that follow: every case's reset
# builds the model with them, and re-attaches the device when its
# descriptors change.
dslogic_options() {
    set -- options "$@"
    emu_cmd "$*" >/dev/null || emu_fail "dslogic: the emulator refused '$*'"
}

# Claims interface 0 of the attached device through usbfs, as another
# program would, until dslogic_release (at most 120 s); as fx2lafw_hold
# in tools/emu/fx2lafw.sh. A claim is no transfer: the model sees
# nothing of it.
dslogic_hold() {
    [ "$EMU_DRY" = 0 ] || return 0 # diffcli counting cases: no device, nothing to claim
    timeout 120 python3 -I -c '
import fcntl, os, signal, struct, sys
d = "/sys/bus/usb/devices/" + sys.argv[1]
bus, dev = (int(open(d + f).read()) for f in ("/busnum", "/devnum"))
fd = os.open("/dev/bus/usb/%03d/%03d" % (bus, dev), os.O_RDWR)
fcntl.ioctl(fd, 0x8004550f, struct.pack("I", 0))  # USBDEVFS_CLAIMINTERFACE
print("held", flush=True)
signal.pause()
' "$EMU_BUSID" >"$TMP/dslogic-hold" 2>&1 &
    HOLDPID=$!
    i=0
    until grep -q '^held$' "$TMP/dslogic-hold"; do
        i=$((i + 1))
        if [ $i -gt 100 ] || ! kill -0 $HOLDPID 2>/dev/null; then
            emu_fail "dslogic: could not claim the interface: $(cat "$TMP/dslogic-hold")"
            return 1
        fi
        sleep 0.1
    done
}

dslogic_release() {
    kill $HOLDPID 2>/dev/null
    wait $HOLDPID 2>/dev/null
    HOLDPID=""
}

emu_cases() {
    D=dreamsourcelab-dslogic
    # A renumerated device's address, in the text the harness's conn=
    # rule does not reach (--show, --get), and a different one for each run.
    C='s/conn(=|: )[0-9]+\.[0-9]+/conn\1BUS.ADDR/g'
    C4=0,1,2,3
    # The CSV comments hold the date.
    V='/^;/d'
    # The address of a renumerated device, alone.
    A="s/'[0-9]+\\.[0-9]+'/'BUS.ADDR'/"
    # The two tools prefix their log lines differently ("sr: DRIVER: "
    # and "oxrok: LEVEL: MODULE: "): the messages are compared without.
    N='s/^sr: [a-z0-9-]+: //; s/^oxrok: [a-z]+: [a-z0-9_:]+: //'
    # A refused option: the tool's line, without what oxrok adds.
    F='/Failed to set device option/!d; s/(invalid argument).*$/\1./'

    # By the address of the session's first attach, the flashed board:
    # the first case, before other options attach it again elsewhere.
    # Without firmware there is no such case: the first tool's upload
    # renumerates the device, which the second tool then cannot reach at
    # EMU_USB, so a full case cannot name it by address.
    emu_case "flashed plus by address" -- -d $D:conn=$EMU_USB --samples 8 -O hex

    # The DSLogic Plus without firmware: every run uploads the FX2
    # firmware in the scan, the device renumerates, and the open loads
    # the FPGA and sets the threshold register (1.4 V).
    dslogic_options profile=plus
    emu_case "plus scan" --keep-header --filter "$C" -- -d $D --scan
    emu_case "plus show" --keep-header --filter "$C" -- -d $D --show
    emu_case "plus by ids" -- -d $D:conn=2a0e.0020 --samples 8 -O hex
    emu_case "plus 1M" -- -d $D -c samplerate=1M --samples 100 -O hex
    emu_case "plus 10k" -- -d $D -c samplerate=10k --samples 64 -O bits
    emu_case "plus 25M" -- -d $D -c samplerate=25M --samples 200 -O hex
    emu_case "plus 100M" -- -d $D -c samplerate=100M --samples 128 -O hex
    # 200 and 400 MHz are the half and quarter modes, divider 1.
    emu_case "plus 200M" -- -d $D -c samplerate=200M --samples 128 -O hex
    emu_case "plus 400M" -- -d $D -C $C4 -c samplerate=400M --samples 128 -O hex
    # Fewer channels: the FPGA sends only the enabled ones, 64 samples
    # of each in turn.
    emu_case "plus 4ch" -- -d $D -C $C4 -c samplerate=2M --samples 300 -O hex
    emu_case "plus sparse channels" -- -d $D -C 0,5,9,15 -c samplerate=5M --samples 200 -O bits
    emu_case "plus one channel" -- -d $D -C 7 -c samplerate=50k --samples 1000 -O hex
    # A high channel toggles slowly, so samples lost off the front of the
    # stream show: the first sample transfer must follow the trigger
    # packet, on every USB backend.
    emu_case "plus high channel" -- -d $D -C 13 --samples 16384 -O hex
    # Three transfers of 40960 bytes (16 channels, 20480 samples each):
    # the header line, samples 20478 to 20481 across the first boundary
    # (sample k is line k + 2 of the CSV without its header, which holds
    # the date) and the last sample; the digest covers all of it.
    emu_case "plus three transfers" --digest --filter '1b; 20480,20483b; $b; d' -- -d $D -c samplerate=20M --samples 50000 -O csv:header=false
    emu_case "plus time limit" -- -d $D -c samplerate=10k --time 10 -O hex
    # Triggers: stage 0 of the setting packet; the FPGA keeps the
    # trigger position's samples (64 at least) before the match.
    emu_case "plus trigger rising" --filter "$V" -- -d $D -c samplerate=1M --samples 200 -t 3=r -O csv:trigger=true
    emu_case "plus trigger falling" --filter "$V" -- -d $D -C $C4 -c samplerate=1M --samples 100 -t 2=f -O csv:trigger=true
    emu_case "plus trigger high" --filter "$V" -- -d $D -C $C4 -c samplerate=1M --samples 100 -t 1=1 -O csv:trigger=true
    emu_case "plus trigger low" --filter "$V" -- -d $D -C $C4 -c samplerate=1M --samples 100 -t 3=0 -O csv:trigger=true
    emu_case "plus trigger edge" --filter "$V" -- -d $D -C $C4 -c samplerate=1M --samples 100 -t 2=e -O csv:trigger=true
    emu_case "plus trigger two matches" --filter "$V" -- -d $D -C $C4 -c samplerate=1M --samples 100 -t 0=0,3=r -O csv:trigger=true
    emu_case "plus trigger ratio" --filter "$V" -- -d $D -C $C4 -c samplerate=1M:captureratio=50 --samples 400 -t 1=r -O csv:trigger=true
    # A trigger on the last sample: libsigrok sends a logic packet of
    # length 0 after it, on which the hex output's length - unitsize
    # wraps and sigrok-cli dies of SIGSEGV; one sample more is a full case.
    emu_case "plus trigger on the last sample" --oracle-bug "protocol.c:861-862 sends a logic packet of length 0, on which output/hex.c:163 wraps" --filter '/Failed to open device/d' -- -d $D -C $C4 --samples 64 -t 0=1 -O hex
    emu_case "plus trigger one sample before the end" -- -d $D -C $C4 --samples 65 -t 0=1 -O hex
    # The packet of length 0 itself is harmless to an output whose loop
    # counts up to the length: the same capture in binary runs full.
    emu_case "plus trigger on the last sample binary" --keep-header --digest --filter 'd' -- -d $D -C $C4 --samples 64 -t 0=1 -O binary
    # (A try of the open during the renumeration may log that it failed,
    # as often as the timing has it: that line goes.)
    # Two letters on a channel are two stages: libsigrok writes both to
    # stage 0 (rising and falling together, which the FPGA reads as
    # rising); oxrok refuses more than the one stage it can keep.
    emu_case "plus trigger two stages" --oracle-differs "protocol.c:386-421 writes the matches of every stage to stage 0, so A then B becomes A and B" --expect-fail --stderr --filter '/Failed to open device/d; s/^oxrok: [a-z]+: [a-z0-9_:]+: //' -- -d $D -C $C4 -c samplerate=1M --samples 100 -t 0=rf -O hex
    # Stream mode: transfers sized for 10 ms at the rate, the trigger
    # position bounded by 10 % of the memory.
    emu_case "plus continuous" -- -d $D -C 0 -c samplerate=10k:continuous=on --samples 10000 -O hex
    # 24576 bytes (12288 samples) a transfer at 1 MHz: the first
    # boundary and the last sample.
    emu_case "plus continuous 16ch" --digest --filter '1b; 12288,12291b; $b; d' -- -d $D -c samplerate=1M:continuous=on --samples 30000 -O csv:header=false
    emu_case "plus continuous trigger" --filter "$V" -- -d $D -C $C4 -c samplerate=100k:continuous=on:captureratio=20 --samples 500 -t 2=r -O csv:trigger=true
    # The external clock and its edge only change the mode bits.
    emu_case "plus external clock" -- -d $D -C $C4 -c external_clock=on --samples 64 -O hex
    emu_case "plus external clock falling" -- -d $D -C $C4 -c external_clock=on:clock_edge=falling --samples 64 -O hex
    # Options read back. libsigrok's clock edge always reads "rising"
    # (config_get() answers signal_edges[0], whatever was set); oxrok's
    # reads back what was set.
    emu_case "plus get samplerate" -- -d $D -c samplerate=50M --get samplerate
    emu_case "plus get clock edge" --oracle-differs "api.c:444 answers signal_edges[0] whatever clock_edge was set to" -- -d $D -c clock_edge=falling --get clock_edge
    emu_case "plus get external clock" -- -d $D -c external_clock=on --get external_clock
    emu_case "plus get continuous" -- -d $D -c continuous=on --get continuous
    emu_case "plus get capture ratio" -- -d $D -c captureratio=30 --get captureratio
    emu_case "plus get limit" -- -d $D -c limit_samples=1234 --get limit_samples
    emu_case "plus get conn" --filter "$A" -- -d $D --get conn
    emu_case "plus get threshold" -- -d $D --get voltage_threshold
    # The threshold register: (low + high) / 2 / 5 V * 255.
    emu_case "plus set threshold" -- -d $D -c voltage_threshold=2.5-3.5 --get voltage_threshold
    emu_case "plus threshold capture" -- -d $D -C $C4 -c voltage_threshold=0.8-0.8 --samples 64 -O hex
    # Above 5 V the DAC value (threshold / 5 V * 255) does not fit its
    # uint8_t: libsigrok converts out of range and writes what comes
    # out (sigrok-cli's stdout, the threshold read back, is kept); oxrok
    # refuses the threshold.
    emu_case "plus threshold above 5V" --oracle-differs "protocol.c:519 converts 10 / 5.0 * 255 to a uint8_t, out of range" --expect-fail --stderr --filter '/Failed to set device option|^[(]/!d; s/(invalid argument).*$/\1./' -- -d $D -c voltage_threshold=10-10 --get voltage_threshold
    emu_case "plus bad clock edge" --expect-fail --stderr --filter "$F" -- -d $D -c clock_edge=sideways --samples 8 -O hex
    emu_case "plus bad rate" --expect-fail --stderr --filter "$F" -- -d $D -c samplerate=7M --samples 8 -O hex
    # Deeper than 16 Msamples at up to 100 MHz: the driver sets RLE
    # mode. One channel, 58 transfers of 282624 samples.
    emu_case "plus rle" --digest --filter '3,$d' -- -d $D -C 0 -c samplerate=100M --samples 16000016 -O hex
    # 95 % of 30 Msamples before the trigger, under the 90 % bound of
    # the 256 Mibit memory: libsigrok computes that bound in 32 bits,
    # which wraps to about 10 % (26843520 samples); oxrok keeps 28499968.
    # The trigger mark's line and where it points.
    emu_case "plus trigger far" --oracle-differs "protocol.c:374-376 computes mem_depth * 90 / 100 in 32 bits, which wraps" --filter '/^T:/!d; =' -- -d $D -C 0 -c samplerate=100M:captureratio=95 --samples 30000000 -t 0=r -O hex

    # The DSLogic Basic: 256 Kibit of memory, so the trigger position
    # is bounded by 90 % of 262144 samples.
    dslogic_options profile=basic
    emu_case "basic scan" --keep-header --filter "$C" -- -d $D --scan
    emu_case "basic 10M" -- -d $D -C $C4 -c samplerate=10M --samples 100 -O hex
    # 95 % of 300000 is bounded by 90 % of the memory: 235904 samples
    # before the trigger (line 235906 of the CSV without its header).
    emu_case "basic trigger far" --digest --filter '1b; 235904,235908b; $b; d' -- -d $D -C 0 -c samplerate=1M:captureratio=95 --samples 300000 -t 0=r -O csv:header=false:trigger=true

    # The DSLogic Pro and the DSCope: their own firmware and bitstream.
    dslogic_options profile=pro
    emu_case "pro scan" --keep-header --filter "$C" -- -d $D --scan
    emu_case "pro 50M" --filter "$V" -- -d $D -C $C4 -c samplerate=50M --samples 100 -t 1=f -O csv:trigger=true
    dslogic_options profile=dscope
    emu_case "dscope scan" --keep-header --filter "$C" -- -d $D --scan
    emu_case "dscope 20M" -- -d $D -C $C4 -c samplerate=20M --samples 100 -O hex

    # The first DSLogic: the threshold is the bitstream. The open loads
    # the 3.3 V one, then writes 1.4 V to the threshold register;
    # libsigrok then reports the 1.4-3.6 V range of the 5 V one, oxrok
    # the 0.7-1.4 V range of the bitstream loaded. Setting a range loads
    # that range's bitstream again.
    T="api.c:355-366 loads the bitstream for threshold 0 (3.3 V, protocol.c:242) and then reports the default 1.4 V as the 5 V bitstream's range"
    dslogic_options profile=dslogic
    emu_case "dslogic scan" --keep-header --filter "$C" -- -d $D --scan
    emu_case "dslogic show" --keep-header --oracle-differs "$T" --filter "$C" -- -d $D --show
    emu_case "dslogic get threshold" --oracle-differs "$T" -- -d $D --get voltage_threshold
    emu_case "dslogic 5V" -- -d $D -C $C4 -c voltage_threshold=1.4-3.6 --samples 64 -O hex
    emu_case "dslogic 3V3" -- -d $D -C $C4 -c voltage_threshold=0.7-1.4 --samples 64 -O hex
    emu_case "dslogic bad threshold" --expect-fail --stderr --filter "$F" -- -d $D -c voltage_threshold=1.0-2.0 --samples 8 -O hex

    # Already flashed: the firmware runs from the start, no upload; the
    # open still loads the FPGA.
    dslogic_options profile=plus flashed=1
    emu_case "flashed plus scan" --keep-header --filter "$C" -- -d $D --scan
    emu_case "flashed plus show" --keep-header --filter "$C" -- -d $D --show
    # Recorded: replayed in cargo test (tests/emu_replay.rs).
    emu_case "flashed plus 1M" --record -- -d $D -C $C4 -c samplerate=1M --samples 100 -O hex
    # In bits, not csv, whose comments hold the date: the fixture keeps the raw stdout.
    emu_case "flashed plus trigger" --record -- -d $D -C $C4 -c samplerate=2M:captureratio=40 --samples 300 -t 2=f -O bits
    emu_case "flashed plus by ids" -- -d $D:conn=2a0e.0020 --samples 8 -O hex

    # Faults, on the flashed board: after an upload the C driver retries
    # a failing open for 3 s, as many times as fit, which would make the
    # transactions depend on wall time. The tools word the rest of
    # stderr differently; the filters keep the line that matters.
    O='/Failed to open device/!d; s/.*/refused/'
    dslogic_options profile=plus flashed=1 stall=version
    emu_case "fault version stall" --expect-fail --stderr --filter "$O" -- -d $D --samples 8 -O hex
    dslogic_options profile=plus flashed=1 stall=revid
    emu_case "fault revid stall" --expect-fail --stderr --filter "$O" -- -d $D --samples 8 -O hex
    dslogic_options profile=plus flashed=1 version=2.0
    emu_case "fault firmware 2.0" --expect-fail --stderr --filter "$O" -- -d $D --samples 8 -O hex
    dslogic_options profile=plus flashed=1 stall=config
    emu_case "fault fpga config stall" --expect-fail --stderr --filter "$O" -- -d $D --samples 8 -O hex
    dslogic_options profile=plus flashed=1 stall_out=fpga
    emu_case "fault bitstream stall" --expect-fail --stderr --filter "$O" -- -d $D --samples 8 -O hex
    dslogic_options profile=plus flashed=1 short_out=fpga
    emu_case "fault bitstream short" --expect-fail --stderr --filter "$O" -- -d $D --samples 8 -O hex
    dslogic_options profile=plus flashed=1 stall=wr_reg
    emu_case "fault threshold stall" --expect-fail --stderr --filter "$O" -- -d $D --samples 8 -O hex
    S='$!d; s/.*/refused/'
    dslogic_options profile=plus flashed=1 stall=stop
    emu_case "fault stop stall" --expect-fail --stderr --filter "$S" -- -d $D --samples 8 -O hex
    dslogic_options profile=plus flashed=1 stall=setting
    emu_case "fault setting stall" --expect-fail --stderr --filter "$S" -- -d $D --samples 8 -O hex
    dslogic_options profile=plus flashed=1 stall_out=setting
    emu_case "fault setting out stall" --expect-fail --stderr --filter "$S" -- -d $D --samples 8 -O hex
    dslogic_options profile=plus flashed=1 short_out=setting
    emu_case "fault setting out short" --expect-fail --stderr --filter "$S" -- -d $D --samples 8 -O hex
    dslogic_options profile=plus flashed=1 stall=start
    emu_case "fault start stall" --expect-fail --stderr --filter "$S" -- -d $D --samples 8 -O hex
    # The trigger packet STALLed or short: trigger_receive() has no branch
    # for it, so libsigrok's capture never ends (the tool is stopped by
    # the timeout); oxrok ends it with an error, the device stopped.
    TB="protocol.c:1010-1026 frees a trigger transfer that STALLed or came short without ending the capture"
    dslogic_options profile=plus flashed=1 trigger=stall
    emu_case "fault trigger stall" --oracle-bug "$TB" --timeout 10 --stderr --filter "/trigger/!d; $N" -- -d $D -C $C4 --samples 16 -O hex
    dslogic_options profile=plus flashed=1 trigger=short
    emu_case "fault trigger short" --oracle-bug "$TB" --timeout 10 --stderr --filter "/trigger/!d; $N" -- -d $D -C $C4 --samples 16 -O hex
    # A trigger that never matches (bit 3 rises on even counts only): the
    # transfer waits. Interrupted after the start (the fifth control write:
    # 0xb3, 0xb8, stop, 0xb4, start), oxrok sends the stop command and ends;
    # sigrok-cli has no SIGINT handler and dies.
    dslogic_options profile=plus flashed=1
    emu_case "interrupt waiting for the trigger" --interrupt out=0:5 --oracle-bug "sigrok-cli has no SIGINT handler, so nothing stops the session" --no-output -- -d $D -C $C4 --samples 16 -t 0=1,3=r -O hex
    # The FPGA stops short of the limit: one transfer of 282624 samples,
    # then nothing. The queue of 10 times out every 125 ms and counts 10
    # empty transfers, so both tools give up after 7 timeouts, with what
    # came. High channels, so that where the data ends shows.
    dslogic_options profile=plus flashed=1 hold=1
    emu_case "fault capture stops short" --digest --filter '3,$d' -- -d $D -C 12 -c samplerate=1M --samples 400000 -O hex
    # The same with a last transfer partly filled, 4096 bytes when it times
    # out: a libusb transfer delivers what it holds (protocol.c:802), 32768
    # samples more.
    dslogic_options profile=plus flashed=1 hold=1 partial=4096
    emu_case "fault capture stops in a partial transfer" --digest --filter '3,$d' -- -d $D -C 12 -c samplerate=1M --samples 400000 -O hex
    # Sample transfers that complete empty, or STALL, count as empty
    # ones; one more than MAX_EMPTY_TRANSFERS (64) in a row and the
    # acquisition ends with no samples.
    dslogic_options profile=plus flashed=1 empty=64
    emu_case "fault 64 empty transfers" -- -d $D -C $C4 --samples 16 -O hex
    dslogic_options profile=plus flashed=1 empty=65
    emu_case "fault 65 empty transfers" --no-output -- -d $D -C $C4 --samples 16 -O hex
    dslogic_options profile=plus flashed=1 stall_in=3
    emu_case "fault sample stall" -- -d $D -C $C4 --samples 16 -O hex
    dslogic_options profile=plus flashed=1 empty=30 stall_in=35
    emu_case "fault 65 empty or stalled" --no-output -- -d $D -C $C4 --samples 16 -O hex
    # A transfer cut mid-atom: libsigrok logs "Invalid transfer
    # length!" and counts the samples by the bytes it has (4 more bytes
    # are 32 samples of the one channel).
    dslogic_options profile=plus flashed=1 short_in=2
    emu_case "fault sample transfer cut" -- -d $D -C 6 -c samplerate=10k:continuous=on --samples 10000 -O hex
    # With three channels those 4 bytes of channel 0 count as 10 samples,
    # and libsigrok reads channels 1 and 2 of them from what the buffer
    # held, here the transfer before (one transfer is queued at this rate);
    # oxrok has them 0.
    dslogic_options profile=plus flashed=1 short_in=2
    emu_case "fault sample transfer cut 3ch" --oracle-differs "protocol.c:732-748 reads the rest of a cut atom from the old buffer" -- -d $D -C 0,1,2 -c samplerate=10k:continuous=on --samples 8000 -O hex
    # The boot loader STALLs the upload: the scan logs it and lists the
    # device all the same; the open then asks the FX2 without firmware
    # for its version, which the boot loader STALLs.
    dslogic_options profile=plus load=cpucs
    emu_case "fault upload cpucs stall" --keep-header --filter "$C" -- -d $D --scan
    U="/firmware version|Firmware upload failed/!d; s/device [0-9]+\.[0-9]+/device BUS.ADDR/; $N"
    emu_case "fault upload cpucs stall capture" --expect-fail --stderr --filter "$U" -- -d $D --samples 8 -O hex
    dslogic_options profile=plus load=chunk
    emu_case "fault upload chunk stall" --expect-fail --stderr --filter "$U" -- -d $D --samples 8 -O hex
    dslogic_options profile=plus load=release
    emu_case "fault upload release stall" --expect-fail --stderr --filter "$U" -- -d $D --samples 8 -O hex
    # Unplugged after the second sample transfer of a capture that
    # wants more: the transfers left fail with NO_DEVICE and the
    # acquisition ends with what came.
    dslogic_options profile=plus flashed=1
    emu_case "unplug mid capture" --unplug in=6:3 -- -d $D -C 0 -c samplerate=10k:continuous=on --samples 20000 -O hex
    # Another program holds the interface: the open reads the version
    # and REVID, then fails to claim it (LIBUSB_ERROR_BUSY).
    if dslogic_hold; then
        emu_case "interface claimed by another program" --expect-fail --stderr --filter "/claim|^Failed to open/!d; $N; s/^(Failed to open device).*\$/\\1./" -- -d $D --samples 8 -O hex
        dslogic_release
    fi
    dslogic_options
}
