# The DreamSourceLab DSLogic family (emu/src/models/dreamsourcelab-dslogic.rs):
# an FX2 without firmware that takes the upload and renumerates, then
# an FPGA that takes its bitstream and a setting packet per capture, as
# the DSLogic Plus (the default), Basic, Pro, the first DSLogic and the
# DSCope, and the same with the firmware already running. Sourced by
# tools/diffcli.sh --emu; see docs/USB.md, "Writing a device model".
# oxrok has no port of the driver yet: every case is an oracle case.
EMU_MODEL=dreamsourcelab-dslogic
EMU_OPTIONS="-o profile=plus"
EMU_DRIVER=dreamsourcelab-dslogic
EMU_GCOV_SHARED="src/usb.c src/ezusb.c"

# The model's options for the cases that follow: every case's reset
# builds the model with them, and re-attaches the device when its
# descriptors change.
dslogic_options() {
    # The emulator trims what it acknowledges: no trailing space.
    set -- options "$@"
    emu_cmd "$*" >/dev/null || emu_fail "dslogic: the emulator refused '$*'"
}

# Claims interface 0 of the attached device through usbfs, as another
# program would, until dslogic_release (at most 120 s); as fx2lafw_hold
# in tools/emu/fx2lafw.sh. A claim is no transfer: the model sees
# nothing of it.
dslogic_hold() {
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
    # A device that renumerated has an address the READY line did not
    # report, and a different one for each run.
    C='s/conn(=|: )[0-9]+\.[0-9]+/conn\1BUS.ADDR/g'
    C4=0,1,2,3
    # The CSV comments hold the date.
    V='/^;/d'
    # The address of a renumerated device, alone.
    A="s/'[0-9]+\\.[0-9]+'/'BUS.ADDR'/"

    # The DSLogic Plus without firmware: every run uploads the FX2
    # firmware in the scan, the device renumerates, and the open loads
    # the FPGA and sets the threshold register (1.4 V).
    dslogic_options profile=plus
    # By bus and address first, while the device is the one READY
    # reported (a renumeration re-attaches it elsewhere): the scan
    # matches the FX2 without firmware, and the open finds the
    # renumerated device by its port.
    emu_case "plus by address" -- -d $D:conn=$EMU_USB --samples 8 -O hex
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
    # Three transfers of 40960 bytes (16 channels, 20480 samples each):
    # the header line, samples 20478 to 20481 across the first boundary
    # (sample k is line k + 6 of the CSV) and the last sample.
    emu_case "plus three transfers" --filter '5b; 20484,20487b; $b; d' -- -d $D -c samplerate=20M --samples 50000 -O csv
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
    # Stream mode: transfers sized for 10 ms at the rate, the trigger
    # position bounded by 10 % of the memory.
    emu_case "plus continuous" -- -d $D -C 0 -c samplerate=10k:continuous=on --samples 10000 -O hex
    # 24576 bytes (12288 samples) a transfer at 1 MHz: the first
    # boundary and the last sample.
    emu_case "plus continuous 16ch" --filter '5b; 12292,12295b; $b; d' -- -d $D -c samplerate=1M:continuous=on --samples 30000 -O csv
    emu_case "plus continuous trigger" --filter "$V" -- -d $D -C $C4 -c samplerate=100k:continuous=on:captureratio=20 --samples 500 -t 2=r -O csv:trigger=true
    # The external clock and its edge only change the mode bits.
    emu_case "plus external clock" -- -d $D -C $C4 -c external_clock=on --samples 64 -O hex
    emu_case "plus external clock falling" -- -d $D -C $C4 -c external_clock=on:clock_edge=falling --samples 64 -O hex
    # Options read back. The clock edge always reads "rising"
    # (config_get() answers signal_edges[0], whatever was set).
    emu_case "plus get samplerate" -- -d $D -c samplerate=50M --get samplerate
    emu_case "plus get clock edge" -- -d $D -c clock_edge=falling --get clock_edge
    emu_case "plus get external clock" -- -d $D -c external_clock=on --get external_clock
    emu_case "plus get continuous" -- -d $D -c continuous=on --get continuous
    emu_case "plus get capture ratio" -- -d $D -c captureratio=30 --get captureratio
    emu_case "plus get limit" -- -d $D -c limit_samples=1234 --get limit_samples
    emu_case "plus get conn" --filter "$A" -- -d $D --get conn
    emu_case "plus get threshold" -- -d $D --get voltage_threshold
    # The threshold register: (low + high) / 2 / 5 V * 255.
    emu_case "plus set threshold" -- -d $D -c voltage_threshold=2.5-3.5 --get voltage_threshold
    emu_case "plus threshold capture" -- -d $D -C $C4 -c voltage_threshold=0.8-0.8 --samples 64 -O hex
    emu_case "plus bad clock edge" --expect-fail --stderr --filter '/Failed to set device option/!d' -- -d $D -c clock_edge=sideways --samples 8 -O hex
    emu_case "plus bad rate" --expect-fail --stderr --filter '/Failed to set device option/!d' -- -d $D -c samplerate=7M --samples 8 -O hex
    # Deeper than 16 Msamples at up to 100 MHz: the driver sets RLE
    # mode. One channel, 58 transfers of 282624 samples.
    emu_case "plus rle" --filter '3,$d' -- -d $D -C 0 -c samplerate=100M --samples 16000016 -O hex

    # The DSLogic Basic: 256 Kibit of memory, so the trigger position
    # is bounded by 90 % of 262144 samples.
    dslogic_options profile=basic
    emu_case "basic scan" --keep-header --filter "$C" -- -d $D --scan
    emu_case "basic 10M" -- -d $D -C $C4 -c samplerate=10M --samples 100 -O hex
    # 95 % of 300000 is bounded by 90 % of the memory: 235904 samples
    # before the trigger (line 235910 of the CSV).
    emu_case "basic trigger far" --filter '/^;/d; 5b; 235908,235912b; $b; d' -- -d $D -C 0 -c samplerate=1M:captureratio=95 --samples 300000 -t 0=r -O csv:trigger=true

    # The DSLogic Pro and the DSCope: their own firmware and bitstream.
    dslogic_options profile=pro
    emu_case "pro scan" --keep-header --filter "$C" -- -d $D --scan
    emu_case "pro 50M" --filter "$V" -- -d $D -C $C4 -c samplerate=50M --samples 100 -t 1=f -O csv:trigger=true
    dslogic_options profile=dscope
    emu_case "dscope scan" --keep-header --filter "$C" -- -d $D --scan
    emu_case "dscope 20M" -- -d $D -C $C4 -c samplerate=20M --samples 100 -O hex

    # The first DSLogic: the threshold is the bitstream. The open loads
    # the 3.3 V one, then writes 1.4 V to the threshold register and
    # reports the 1.4-3.6 V range of the 5 V one; setting a range loads
    # that range's bitstream again.
    dslogic_options profile=dslogic
    emu_case "dslogic scan" --keep-header --filter "$C" -- -d $D --scan
    emu_case "dslogic show" --keep-header --filter "$C" -- -d $D --show
    emu_case "dslogic get threshold" -- -d $D --get voltage_threshold
    emu_case "dslogic 5V" -- -d $D -C $C4 -c voltage_threshold=1.4-3.6 --samples 64 -O hex
    emu_case "dslogic 3V3" -- -d $D -C $C4 -c voltage_threshold=0.7-1.4 --samples 64 -O hex
    emu_case "dslogic bad threshold" --expect-fail --stderr --filter '/Failed to set device option/!d' -- -d $D -c voltage_threshold=1.0-2.0 --samples 8 -O hex

    # Already flashed: the firmware runs from the start, no upload; the
    # open still loads the FPGA.
    dslogic_options profile=plus flashed=1
    emu_case "flashed plus scan" --keep-header --filter "$C" -- -d $D --scan
    emu_case "flashed plus show" --keep-header --filter "$C" -- -d $D --show
    emu_case "flashed plus 1M" -- -d $D -C $C4 -c samplerate=1M --samples 100 -O hex
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
    # length!" and takes the whole words it has.
    dslogic_options profile=plus flashed=1 short_in=2
    emu_case "fault sample transfer cut" -- -d $D -C 6 -c samplerate=10k:continuous=on --samples 10000 -O hex
    # The boot loader STALLs the upload: the scan logs it and lists the
    # device all the same; the open then asks the FX2 without firmware
    # for its version, which the boot loader STALLs.
    dslogic_options profile=plus load=cpucs
    emu_case "fault upload cpucs stall" --keep-header --filter "$C" -- -d $D --scan
    emu_case "fault upload cpucs stall capture" --expect-fail --stderr --filter '/firmware version|Firmware upload failed/!d; s/device [0-9]+\.[0-9]+/device BUS.ADDR/' -- -d $D --samples 8 -O hex
    dslogic_options profile=plus load=chunk
    emu_case "fault upload chunk stall" --expect-fail --stderr --filter '/firmware version|Firmware upload failed/!d; s/device [0-9]+\.[0-9]+/device BUS.ADDR/' -- -d $D --samples 8 -O hex
    dslogic_options profile=plus load=release
    emu_case "fault upload release stall" --expect-fail --stderr --filter '/firmware version|Firmware upload failed/!d; s/device [0-9]+\.[0-9]+/device BUS.ADDR/' -- -d $D --samples 8 -O hex
    # Unplugged after the second sample transfer of a capture that
    # wants more: the transfers left fail with NO_DEVICE and the
    # acquisition ends with what came.
    dslogic_options profile=plus flashed=1
    emu_case "unplug mid capture" --unplug in=6:3 -- -d $D -C 0 -c samplerate=10k:continuous=on --samples 20000 -O hex
    # Another program holds the interface: the open reads the version
    # and REVID, then fails to claim it (LIBUSB_ERROR_BUSY).
    if dslogic_hold; then
        emu_case "interface claimed by another program" --expect-fail --stderr --filter '/claim/!d' -- -d $D --samples 8 -O hex
        dslogic_release
    fi
    dslogic_options
}
