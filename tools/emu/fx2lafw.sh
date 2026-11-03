# The fx2lafw logic analyzer (emu/src/models/fx2lafw.rs): an FX2
# without firmware that takes the upload and renumerates, as the bare
# Cypress board (16 channels), a Saleae Logic (8 channels) and sigrok's
# fx2grok-flat (1d50:608c, 8 channels), and the same boards with the
# firmware already running. Sourced by tools/diffcli.sh --emu; see
# docs/USB.md, "Writing a device model".
EMU_MODEL=fx2lafw
EMU_OPTIONS="-o profile=cypress-fx2"
EMU_DRIVER=fx2lafw
EMU_GCOV_SHARED="src/usb.c src/ezusb.c"
# oxrok has the driver: its cases must run full.
EMU_FULL=fx2lafw

# The model's options for the cases that follow: every case's reset
# builds the model with them, and re-attaches the device when its
# descriptors change.
fx2lafw_options() {
    # The emulator trims what it acknowledges: no trailing space.
    set -- options "$@"
    emu_cmd "$*" >/dev/null || emu_fail "fx2lafw: the emulator refused '$*'"
}

# Claims interface 0 of the attached device through usbfs, as another
# program would, and holds it until fx2lafw_release (at most 120 s, so
# an interrupted run leaves nothing behind). The device must not be
# re-attached meanwhile: the cases between keep its descriptors. A claim
# is no transfer, so the model sees nothing of it.
fx2lafw_hold() {
    timeout 120 python3 -I -c '
import fcntl, os, signal, struct, sys
d = "/sys/bus/usb/devices/" + sys.argv[1]
bus, dev = (int(open(d + f).read()) for f in ("/busnum", "/devnum"))
fd = os.open("/dev/bus/usb/%03d/%03d" % (bus, dev), os.O_RDWR)
fcntl.ioctl(fd, 0x8004550f, struct.pack("I", 0))  # USBDEVFS_CLAIMINTERFACE
print("held", flush=True)
signal.pause()
' "$EMU_BUSID" >"$TMP/fx2lafw-hold" 2>&1 &
    HOLDPID=$!
    i=0
    until grep -q '^held$' "$TMP/fx2lafw-hold"; do
        i=$((i + 1))
        if [ $i -gt 100 ] || ! kill -0 $HOLDPID 2>/dev/null; then
            emu_fail "fx2lafw: could not claim the interface: $(cat "$TMP/fx2lafw-hold")"
            return 1
        fi
        sleep 0.1
    done
}

fx2lafw_release() {
    kill $HOLDPID 2>/dev/null
    wait $HOLDPID 2>/dev/null
    HOLDPID=""
}

emu_cases() {
    F=fx2lafw
    D8=D0,D1,D2,D3,D4,D5,D6,D7
    # A device that renumerated has an address the READY line did not
    # report, and a different one for each tool.
    C='s/conn(=|: )[0-9]+\.[0-9]+/conn\1BUS.ADDR/g'

    # The bare Cypress FX2: every run uploads the firmware first, in the
    # scan, and the device renumerates before it is opened.
    fx2lafw_options profile=cypress-fx2
    emu_case "fx2 scan" --keep-header -- -d $F --scan
    emu_case "fx2 show" --keep-header --filter "$C" -- -d $F --show
    emu_case "fx2 by ids" -- -d $F:conn=04b4.8613 --samples 8 -O hex
    emu_case "fx2 8ch 20k" -- -d $F -C $D8 -c samplerate=20k --samples 600 -O hex
    emu_case "fx2 8ch 1M" -- -d $F -C $D8 -c samplerate=1M --samples 64 -O bits
    emu_case "fx2 8ch 3M" --filter '1,/^D7/!d' -- -d $F -C $D8 -c samplerate=3M --samples 40000 -O hex
    emu_case "fx2 8ch 24M" -- -d $F -C $D8 -c samplerate=24M --samples 1024 -O hex
    emu_case "fx2 8ch 48M" -- -d $F -C $D8 -c samplerate=48M --samples 100 -O hex
    emu_case "fx2 16ch 50k" -- -d $F -c samplerate=50k --samples 300 -O hex
    emu_case "fx2 16ch 12M" --filter '1,/^D15/!d' -- -d $F -c samplerate=12M --samples 2000 -O bits
    emu_case "fx2 one sample" -- -d $F -C $D8 -c samplerate=1M --samples 1 -O bits
    # Triggers on the 16 channel board with all channels: with fewer,
    # libsigrok's soft trigger takes two bytes per sample (all logic
    # channels) while the stream has one, see the model's comments.
    emu_case "fx2 trigger 16ch" -- -d $F -c samplerate=1M --samples 24 -t D9=r -O bits
    emu_case "fx2 trigger 16ch ratio" -- -d $F -c samplerate=500k:captureratio=50 --samples 40 -t D3=f -O bits
    emu_case "fx2 frames" -- -d $F -C $D8 -c samplerate=1M:limit_frames=2 --samples 8 -O bits
    emu_case "fx2 probe names" --keep-header -- -d $F:probe_names=CLK,DATA,CS --scan
    emu_case "fx2 bad rate" --expect-fail --stderr --filter '1!d; s/.*/refused/' -- -d $F -c samplerate=7M --samples 8 -O hex
    # Several transfers of 512 bytes (10 ms at 20 kHz): a trigger in the
    # third, a time limit (sigrok-cli makes it a sample limit).
    emu_case "fx2 16ch 20k trigger late" -- -d $F -c samplerate=20k --samples 300 -t D9=r -O hex
    emu_case "fx2 time limit" -- -d $F -c samplerate=50k --time 30 -O hex
    emu_case "fx2 16ch at 24M" --expect-fail --stderr --filter '/Unable to sample|maximum sample rate/!d; s/.*/refused/' -- -d $F -c samplerate=24M --samples 16 -O hex

    # A Saleae Logic, 8 channels.
    fx2lafw_options profile=saleae-logic
    emu_case "logic scan" --keep-header -- -d $F --scan
    emu_case "logic 25k" -- -d $F -c samplerate=25k --samples 100 -O hex
    emu_case "logic 24M" --filter '1,/^D7/!d' -- -d $F -c samplerate=24M --samples 3000 -O hex
    emu_case "logic trigger rising" -- -d $F -c samplerate=1M --samples 32 -t D5=r -O bits
    emu_case "logic trigger falling" -- -d $F -c samplerate=4M --samples 20 -t D4=f -O bits
    emu_case "logic trigger ratio" -- -d $F -c samplerate=1M:captureratio=25 --samples 32 -t D6=1 -O bits
    emu_case "logic trigger two matches" -- -d $F -c samplerate=200k --samples 16 -t D0=0,D7=f -O bits
    emu_case "logic trigger frames" -- -d $F -c samplerate=1M:limit_frames=3 --samples 6 -t D2=r -O bits
    emu_case "logic 20k six transfers" -- -d $F -c samplerate=20k --samples 3000 -O hex
    emu_case "logic frames across transfers" -- -d $F -c samplerate=20k:limit_frames=3 --samples 300 -O hex
    emu_case "logic trigger frames across transfers" -- -d $F -c samplerate=20k:limit_frames=3 --samples 200 -t D7=f -O hex

    # sigrok's fx2grok-flat: the same ids before and after the upload.
    fx2lafw_options profile=sigrok-fx2-8ch
    emu_case "fx2grok scan" --keep-header -- -d $F --scan
    emu_case "fx2grok 2M" -- -d $F -c samplerate=2M --samples 50 -O hex

    # Already flashed: the firmware runs from the start, no upload.
    fx2lafw_options profile=cypress-fx2 flashed=1
    emu_case "flashed fx2 scan" --keep-header --filter "$C" -- -d $F --scan
    emu_case "flashed fx2 show" --keep-header --filter "$C" -- -d $F --show
    emu_case "flashed fx2 8ch 2M" --record -- -d $F -C $D8 -c samplerate=2M --samples 128 -O hex
    emu_case "flashed fx2 16ch 6M" -- -d $F -c samplerate=6M --samples 100 -O hex
    fx2lafw_options profile=saleae-logic flashed=1
    emu_case "flashed logic scan" --keep-header --filter "$C" -- -d $F --scan
    emu_case "flashed logic 16M" --record -- -d $F -c samplerate=16M --samples 50 -t D2=r -O bits
    # A frame that ends where a transfer ends: libsigrok's
    # receive_transfer() then hands its NULL soft trigger to
    # soft_trigger_logic_check() (protocol.c:484) and crashes. On the
    # flashed board, as the stderr golden must not depend on whether
    # libusb opened the renumerated device before its node was
    # accessible (the C driver logs that and tries again).
    emu_case "logic frame on a transfer boundary" --oracle-bug "receive_transfer() crashes when a frame without trigger ends at the end of a transfer" -- -d $F -c samplerate=20k:limit_frames=2 --samples 512 -O hex
    fx2lafw_options profile=sigrok-fx2-8ch flashed=1
    emu_case "flashed fx2grok by ids" --filter "$C" -- -d $F:conn=1d50.608c --samples 8 -O hex

    # The CWAV USBee AX: 8 channels and the analog input A0
    # (DEV_CAPS_AX_ANALOG). With A0 enabled the samples are 16 bit, the
    # logic byte then the ADC's, the start sets the CTL2 clock, and the
    # driver refuses rates above 12 MHz; without it, an 8 channel board.
    # The model's ADC ramp reads n / 12.8 V at sample n (to 127).
    # libsigrok's soft trigger is not compared with A0 enabled: it takes
    # one byte per sample (the logic channels) on a stream of two, so it
    # matches the ADC's bytes too and starts the data at twice the
    # offset, and sigrok-cli still exits 0, which --oracle-bug cannot
    # take (see the model's comments).
    fx2lafw_options profile=usbee-ax
    emu_case "ax scan" --keep-header -- -d $F --scan
    emu_case "ax a0 1M" -- -d $F -C A0 -c samplerate=1M --samples 200 -O analog
    emu_case "ax 8ch 24M" -- -d $F -C $D8 -c samplerate=24M --samples 1024 -O hex
    fx2lafw_options profile=usbee-ax flashed=1
    emu_case "flashed ax scan" --keep-header --filter "$C" -- -d $F --scan
    emu_case "flashed ax show" --keep-header --filter "$C" -- -d $F --show
    emu_case "ax probe names" --keep-header --filter "$C" -- -d $F:probe_names=CLK,DATA --scan
    emu_case "ax by ids" -- -d $F:conn=08a9.0014 -C A0 --samples 8 -O analog
    emu_case "ax logic and a0 2M" --record -- -d $F -c samplerate=2M --samples 64 -O bits
    emu_case "ax d0 d1 a0 1M" -- -d $F -C D0,D1,A0 -c samplerate=1M --samples 40 -O bits
    # A0 alone in CSV. sigrok-cli also feeds the packets the csv output
    # answers nothing for to an analog output (docs/CLI.md): its
    # FRAME-END and the blank line of the frame that begins after the
    # limit go.
    emu_case "ax a0 csv" --filter '/^FRAME-END$/d; /^$/d' -- -d $F -C A0 -c samplerate=500k --samples 300 -O csv:header=false
    # Three transfers of 256 samples at 20 kHz, each block of the ramp
    # one step above the last; every 50th line kept, all of it digested.
    emu_case "ax a0 20k transfers" --digest --filter '1~50!d' -- -d $F -C A0 -c samplerate=20k --samples 700 -O analog
    emu_case "ax a0 12M" --digest --filter '1~997!d' -- -d $F -c samplerate=12M --samples 30000 -O analog
    emu_case "ax a0 frames" -- -d $F -C A0 -c samplerate=1M:limit_frames=2 --samples 3 -O analog
    emu_case "ax trigger without a0" -- -d $F -C $D8 -c samplerate=1M --samples 16 -t D3=r -O bits
    # 16 bit samples go to 12 MHz: A0 refuses the faster rates,
    # command_start_acquisition() after the transfers were submitted.
    for r in 16M 24M 48M; do
        emu_case "ax a0 at $r" --expect-fail --stderr --filter '/Unable to sample/!d; s/.*/refused/' -- -d $F -C A0 -c samplerate=$r --samples 8 -O analog
    done
    emu_case "ax no channel a1" --expect-fail --stderr --filter '/unknown channel/!d; s/.*/refused/' -- -d $F -C A1 --samples 8 -O analog
    # config_list() answers SR_ERR_NA for any channel group.
    emu_case "ax a0 group get" --expect-fail --stderr --filter '/samplerate/!d; s/.*/refused/' -- -d $F -g A0 --get samplerate

    # Faults, on the flashed board: after an upload the C driver retries
    # a failing open for 3 s, as many times as fit, which would make the
    # transactions depend on wall time. The last line of stderr is the
    # tool's own (sigrok-cli and oxrok word the rest differently).
    O='/^Failed to open device/!d; s/.*/refused/'
    fx2lafw_options profile=saleae-logic flashed=1 stall=version
    emu_case "fault version stall" --expect-fail --stderr --filter "$O" -- -d $F --samples 8 -O hex
    fx2lafw_options profile=saleae-logic flashed=1 stall=revid
    emu_case "fault revid stall" --expect-fail --stderr --filter "$O" -- -d $F --samples 8 -O hex
    fx2lafw_options profile=saleae-logic flashed=1 stall=start
    emu_case "fault start stall" --expect-fail --stderr --filter '$!d; s/.*/refused/' -- -d $F --samples 8 -O hex
    # A firmware of another major version, which knows no start command
    # of this one: libsigrok logs the mismatch but opens the device all
    # the same, then fails at the start.
    fx2lafw_options profile=saleae-logic flashed=1 version=2.0 stall=start
    emu_case "fault firmware 2.0" --expect-fail --stderr --filter "$O" --oracle-bug "fx2lafw_dev_open() logs a firmware major version mismatch but returns SR_OK (ret holds the REVID read's)" -- -d $F --samples 8 -O hex
    # Transfers that complete empty, or STALL, count as empty ones; one
    # more than MAX_EMPTY_TRANSFERS (64) in a row and the FX2 "gave up":
    # the acquisition ends with no samples.
    fx2lafw_options profile=saleae-logic flashed=1 empty=64
    emu_case "fault 64 empty transfers" -- -d $F -c samplerate=1M --samples 16 -O hex
    fx2lafw_options profile=saleae-logic flashed=1 empty=65
    emu_case "fault 65 empty transfers" --no-output -- -d $F -c samplerate=1M --samples 16 -O hex
    fx2lafw_options profile=saleae-logic flashed=1 stall_in=3
    emu_case "fault bulk in stall" -- -d $F -c samplerate=1M --samples 16 -O hex
    fx2lafw_options profile=saleae-logic flashed=1 empty=30 stall_in=35
    emu_case "fault 65 empty or stalled" --no-output -- -d $F -c samplerate=1M --samples 16 -O hex
    # The string reads of the scan failing (api.c 255-288): a STALL, or
    # an answer shorter than its bLength (libusb: LIBUSB_ERROR_IO), of
    # the language table, the manufacturer, the product or the serial
    # number. The device is skipped, so the scan finds nothing. The
    # options change no descriptor: no re-attach. The tools word the
    # error each its own way; which string failed is compared.
    S='s/^.*(Failed to get [a-z ]+ string descriptor).*/\1/'
    for f in stall_string=0 short_string=0 stall_string=1 short_string=2 stall_string=3; do
        fx2lafw_options profile=saleae-logic flashed=1 $f
        emu_case "fault $f scan" --stderr --filter "$S" -- -d $F --scan
    done
    # Unplugged mid capture (the 3rd bulk IN of six): the transfers
    # still queued fail with LIBUSB_TRANSFER_NO_DEVICE and the
    # acquisition ends with what came before (protocol.c 444-447).
    fx2lafw_options profile=saleae-logic flashed=1
    emu_case "unplug mid capture" --unplug in=2:3 -- -d $F -c samplerate=20k --samples 3000 -O hex
    # Another program holds the interface: both tools read the version
    # and REVID, then fail to claim it (LIBUSB_ERROR_BUSY). The same
    # descriptors as above, so the device is not re-attached.
    fx2lafw_options profile=saleae-logic flashed=1
    if fx2lafw_hold; then
        emu_case "interface claimed by another program" --expect-fail --stderr --filter "$O" -- -d $F --samples 8 -O hex
        fx2lafw_release
    fi
    fx2lafw_options
}
