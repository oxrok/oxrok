# The Saleae Logic16 (emu/src/models/saleae-logic16.rs): an FX2 that
# takes its firmware and renumerates, then an FPGA bitstream through
# scrambled commands on endpoint 1. Sourced by tools/diffcli.sh --emu;
# see docs/USB.md, "Writing a device model".
EMU_MODEL=saleae-logic16
EMU_DRIVER=saleae-logic16
EMU_GCOV_SHARED="src/usb.c src/ezusb.c"
# The session attaches the board with its firmware running, so that the
# first case can name it by the address of that attach (EMU_USB): a
# board without firmware renumerates under the first tool and is
# attached again, at another address, for the second.
EMU_OPTIONS="-o flashed=1"
# oxrok has the driver: its cases must run full.
EMU_FULL=saleae-logic16

# The model's options for the cases that follow (see fx2lafw.sh).
logic16_options() {
    set -- options "$@"
    emu_cmd "$*" >/dev/null || emu_fail "saleae-logic16: the emulator refused '$*'"
}

# Claims interface 0 through usbfs as another program would, until
# logic16_release (as fx2lafw_hold in fx2lafw.sh).
logic16_hold() {
    [ "$EMU_DRY" = 0 ] || return 0 # diffcli counting cases: no device, nothing to claim
    timeout 120 python3 -I -c '
import fcntl, os, signal, struct, sys
d = "/sys/bus/usb/devices/" + sys.argv[1]
bus, dev = (int(open(d + f).read()) for f in ("/busnum", "/devnum"))
fd = os.open("/dev/bus/usb/%03d/%03d" % (bus, dev), os.O_RDWR)
fcntl.ioctl(fd, 0x8004550f, struct.pack("I", 0))  # USBDEVFS_CLAIMINTERFACE
print("held", flush=True)
signal.pause()
' "$EMU_BUSID" >"$TMP/logic16-hold" 2>&1 &
    HOLDPID=$!
    i=0
    until grep -q '^held$' "$TMP/logic16-hold"; do
        i=$((i + 1))
        if [ $i -gt 100 ] || ! kill -0 $HOLDPID 2>/dev/null; then
            emu_fail "saleae-logic16: could not claim the interface: $(cat "$TMP/logic16-hold")"
            return 1
        fi
        sleep 0.1
    done
}

logic16_release() {
    kill $HOLDPID 2>/dev/null
    wait $HOLDPID 2>/dev/null
    HOLDPID=""
}

emu_cases() {
    F=saleae-logic16
    # A renumerated device's address, in the text the harness's conn=
    # rule does not reach (--show, --get).
    C='s/conn(=|: )[0-9]+\.[0-9]+/conn\1BUS.ADDR/g'
    D4=0,1,2,3
    # The two tools prefix their log lines differently ("sr: DRIVER: "
    # and "oxrok: LEVEL: MODULE: "): the messages are compared without.
    N='s/^sr: [a-z0-9-]+: //; s/^oxrok: [a-z]+: [a-z0-9_:]+: //'
    # The tool's own line when the open fails; oxrok adds the reason.
    O='s/^(Failed to open device).*$/\1./'
    # A capture that fails to start. The C driver logs nothing of its
    # own there: sigrok-cli's two lines are libsigrok's session ("Could
    # not start saleae-logic16 device usb/... acquisition.") and its own
    # "Failed to start session."; oxrok's session hands back the
    # driver's error, which sigrok-cli never prints. No line of one tool
    # has a counterpart in the other, so the last line is reduced to
    # where the run failed: at the open, or at the start.
    R='$!d; /^Failed to open device/{s/.*/Failed to open device./;b}; s/.*/start refused/'
    # A refused rate: the message alone.
    S="/Unable to sample/!d; $N"

    # By the address of the session's first attach, the flashed board:
    # the first case, before other options attach it again elsewhere.
    emu_case "flashed by address" -- -d $F:conn=$EMU_USB -C $D4 --samples 16 -O bits

    # Without firmware: every run uploads it in the scan, and the
    # device renumerates before it is opened.
    logic16_options flashed=0
    emu_case "scan" --keep-header -- -d $F --scan
    emu_case "bad conn" --stderr --filter "$N" -- -d $F:conn=nonsense --scan
    emu_case "show" --keep-header --filter "$C" -- -d $F --show
    emu_case "by ids" -- -d $F:conn=21a9.1001 -C $D4 --samples 16 -O bits
    emu_case "4ch 1M" -- -d $F -C $D4 -c samplerate=1M --samples 64 -O bits
    emu_case "16ch 500k" --filter '1,/^15:/!d' -- -d $F --samples 20000 -O bits
    emu_case "16ch 16M" --filter '1,/^15:/!d' -- -d $F -c samplerate=16M --samples 3000 -O bits
    emu_case "1ch 100M" -- -d $F -C 5 -c samplerate=100M --samples 2000 -O hex
    emu_case "3ch 100M" -- -d $F -C 0,8,15 -c samplerate=100M --samples 100 -O bits
    emu_case "2ch 80M" -- -d $F -C 3,4 -c samplerate=80M --samples 100 -O bits
    emu_case "2ch 32M" -- -d $F -C 0,1 -c samplerate=32M --samples 40 -O bits
    # sigrok-cli reads "12.5M" as 12 Hz; 12500k is 12.5 MHz.
    emu_case "8ch 12.5M" -- -d $F -C 0,1,2,3,4,5,6,7 -c samplerate=12500k --samples 40 -O hex
    emu_case "5V threshold" -- -d $F -C $D4 -c voltage_threshold=1.4-3.6 --samples 32 -O bits
    emu_case "get threshold" -- -d $F --get voltage_threshold
    emu_case "get samplerate" -- -d $F --get samplerate
    emu_case "get captureratio" -- -d $F --get captureratio
    emu_case "trigger rising" -- -d $F -C $D4 -c samplerate=1M --samples 32 -t 3=r -O bits
    emu_case "trigger ratio" -- -d $F -C $D4 -c samplerate=2M:captureratio=50 --samples 40 -t 2=f -O bits
    # A ratio above 100: api.c:473 would take it, but sr_config_set()
    # refuses it first (hwdriver.c:757-764), and so does oxrok.
    emu_case "trigger ratio above 100" --expect-fail --stderr --filter "/LIBUSB_ERROR_ACCESS/d; $N; s/(invalid argument).*\$/\1./" -- -d $F -C $D4 -c samplerate=2M:captureratio=150 --samples 40 -t 2=f -O bits
    emu_case "trigger two" -- -d $F -C 0,1,9 -c samplerate=4M --samples 24 -t 0=0,9=r -O bits
    emu_case "trigger late" --filter '1,/^4:/!d' -- -d $F -C 0,1,2,3,4 -c samplerate=500k --samples 40 -t 4=e -O bits
    # Rates off the list the driver takes all the same: 100 MHz / 250
    # and 160 MHz / 250.
    emu_case "400k unlisted" -- -d $F -C 0,1 -c samplerate=400k --samples 40 -O bits
    emu_case "640k unlisted" -- -d $F -C 0,1 -c samplerate=640k --samples 40 -O bits
    emu_case "time limit" -- -d $F -C 0 -c samplerate=1M --time 1 -O hex
    emu_case "bad rate" --expect-fail --stderr --filter "$S" -- -d $F -C 0 -c samplerate=3M --samples 8 -O hex
    # A rate at which the channels carry less than 8000 bytes a second:
    # bytes_per_ms() is 0 and get_number_of_transfers() (api.c:544)
    # divides by it before logic16_setup_acquisition() can refuse the
    # rate. sigrok-cli dies of SIGFPE, printing nothing; oxrok refuses the
    # rate.
    emu_case "low rate crash" --oracle-bug "api.c:544 divides by get_buffer_size(), 0 under 8000 bytes a second" --expect-fail --stderr --filter "$S" -- -d $F -C 0 -c samplerate=1k --samples 8 -O hex
    emu_case "rate above 100M" --expect-fail --stderr --filter "$S" -- -d $F -C 0 -c samplerate=200M --samples 8 -O hex
    # Only the last line: libusb may log LIBUSB_ERROR_ACCESS first for the
    # renumerated device (docs/USB.md).
    emu_case "bad threshold" --expect-fail --stderr --filter '/Failed to set/!d; s/(invalid argument).*$/\1./' -- -d $F -C 0 -c voltage_threshold=1.0-2.0 --samples 8 -O hex
    emu_case "too many channels" --expect-fail --stderr --filter "$S" -- -d $F -C $D4 -c samplerate=100M --samples 8 -O hex

    # The firmware already running: no upload, and the scan reads the
    # strings.
    logic16_options flashed=1
    emu_case "flashed scan" --keep-header --filter "$C" -- -d $F --scan
    emu_case "flashed get conn" --filter "s/'[0-9]+[.][0-9]+'/'BUS.ADDR'/" -- -d $F --get conn
    emu_case "flashed 4ch 2M" -- -d $F -C $D4 -c samplerate=2M --samples 48 -O bits
    logic16_options flashed=1 bitstream=new
    emu_case "new bitstream" -- -d $F -C $D4 -c samplerate=5M --samples 48 -O bits
    emu_case "new bitstream 5V" -- -d $F -C 6,7 -c samplerate=8M:voltage_threshold=1.4-3.6 --samples 32 -O bits
    logic16_options flashed=1 variant=mcupro
    # Recorded: the clone's FPGA takes no bitstream, so the replay needs
    # no firmware file.
    emu_case "mcupro" --record -- -d $F -C $D4 -c samplerate=10M --samples 48 -O bits
    emu_case "mcupro trigger" --record -- -d $F -C $D4 -c samplerate=10M:captureratio=25 --samples 64 -t 2=f -O bits
    emu_case "mcupro 5V" -- -d $F -C 0 -c samplerate=20M:voltage_threshold=1.4-3.6 --samples 32 -O bits
    logic16_options flashed=1
    if logic16_hold; then
        emu_case "interface claimed by another program" --expect-fail --stderr --filter "$N; $O" -- -d $F --samples 8 -O hex
        logic16_release
    fi

    # Faults, on the flashed board: after an upload the driver retries
    # a failing open for 3 s, which would make the transactions depend
    # on wall time.
    for f in abort-sync read-eeprom fpga-write upload-init upload-data led-table led-mode; do
        logic16_options flashed=1 stall=$f
        emu_case "fault stall $f" --expect-fail --stderr --filter "$N; $O" -- -d $F -C 0 --samples 8 -O hex
    done
    for n in 2 3 4 5; do
        logic16_options flashed=1 stall=fpga-read:$n
        emu_case "fault stall fpga read $n" --expect-fail --stderr --filter "$N; $O" -- -d $F -C 0 --samples 8 -O hex
    done
    logic16_options flashed=1 stall=fpga-read:1
    emu_case "fault mcupro probe stalls" -- -d $F -C 0 --samples 8 -O hex
    logic16_options flashed=1 stall=read-eeprom:2
    emu_case "fault stall primer eeprom" --expect-fail --stderr --filter "$N; $O" -- -d $F -C 0 --samples 8 -O hex
    logic16_options flashed=1 stall=fpga-write:17
    emu_case "fault stall primer end" --expect-fail --stderr --filter "$N; $O" -- -d $F -C 0 --samples 8 -O hex
    for n in 6 7 8; do
        logic16_options flashed=1 stall=fpga-read:$n
        emu_case "fault stall setup read $n" --expect-fail --stderr --filter "$R" -- -d $F -C 0 --samples 8 -O hex
    done
    for n in 18 19 20 21 22 23 24 25; do
        logic16_options flashed=1 stall=fpga-write:$n
        emu_case "fault stall setup write $n" --expect-fail --stderr --filter "$R" -- -d $F -C 0 --samples 8 -O hex
    done
    logic16_options flashed=1 stall=start
    emu_case "fault stall start" --expect-fail --stderr --filter "$R" -- -d $F -C 0 --samples 8 -O hex
    logic16_options flashed=1 stall=abort
    emu_case "fault stall abort" -- -d $F -C 0 --samples 8 -O hex
    for n in 9 10 11; do
        logic16_options flashed=1 stall=fpga-read:$n
        emu_case "fault stall stop read $n" -- -d $F -C 0 --samples 8 -O hex
    done
    logic16_options flashed=1 stall=fpga-write:26
    emu_case "fault stall stop write" -- -d $F -C 0 --samples 8 -O hex
    logic16_options flashed=1 stall_reply=abort-sync
    emu_case "fault stall reply" --expect-fail --stderr --filter "$N; $O" -- -d $F -C 0 --samples 8 -O hex
    logic16_options flashed=1 short_reply=read-eeprom
    emu_case "fault short reply" --expect-fail --stderr --filter "$N; $O" -- -d $F -C 0 --samples 8 -O hex
    logic16_options flashed=1 abort_reply=0x55
    emu_case "fault abort reply" --expect-fail --stderr --filter "$N; $O" -- -d $F -C 0 --samples 8 -O hex
    # The driver's warnings and debug lines that show each fault took
    # effect (it carries on after all of them).
    # -l 4 lines carry a timestamp, which goes.
    W="/Unsupported FPGA|Invalid state|FIFO overflow/!d; s/ \[[0-9:.]+\]//; $N"
    logic16_options flashed=1 fpga_version=0x22
    emu_case "fault fpga version" --stderr --filter "$W" -- -l 4 -d $F -C 0 --samples 8 -O hex
    logic16_options flashed=1 status_xor=0x08
    emu_case "fault status bits" --stderr --filter "$W" -- -l 4 -d $F -C 0 --samples 8 -O hex
    logic16_options flashed=1 mode_xor=0x01
    emu_case "fault mode bits" --stderr --filter "$W" -- -l 4 -d $F -C 0 --samples 8 -O hex
    logic16_options flashed=1 overflow=1
    emu_case "fault overflow" --stderr --filter "$W" -- -l 4 -d $F -C 0 --samples 8 -O hex
    logic16_options flashed=1 empty=64
    emu_case "fault 64 empty transfers" -- -d $F -C 0 --samples 8 -O hex
    logic16_options flashed=1 empty=65
    emu_case "fault 65 empty transfers" --oxrok-fails "over 64 sample transfers in a row failed or brought nothing" --no-output -- -d $F -C 0 --samples 8 -O hex
    logic16_options flashed=1 stall_in=2
    emu_case "fault bulk in stall" -- -d $F -C 0 --samples 8 -O hex
    logic16_options flashed=1 short_in=1
    emu_case "fault odd length" --oxrok-fails "over 64 sample transfers in a row failed or brought nothing" --no-output -- -d $F -C 0 --samples 8 -O hex
    # One word with 16 channels: no complete sample yet, the driver
    # resubmits and the stream goes on.
    logic16_options flashed=1 short_in=2
    emu_case "fault short transfer" --filter '1,/^15:/!d' -- -d $F --samples 600 -O bits
    logic16_options flashed=1 stall=upload-init:2
    emu_case "fault stall second upload" --expect-fail --stderr --filter "$R" -- -d $F -C 0 -c voltage_threshold=1.4-3.6 --samples 8 -O hex
    logic16_options flashed=1
    emu_case "unplug mid capture" --oxrok-fails "device disconnected" --unplug in=2:2 --stderr --filter "$N" -- -d $F -C 0,1 -c samplerate=1M --samples 100000 -O hex
    # Unplugged during the firmware upload in the scan (the Nth control
    # write: SET_CONFIGURATION, the reset hold, the 2 chunks, the
    # release): the scan still lists the device, which is gone when
    # opened. Whether open still finds it, or no longer lists it, depends
    # on when the kernel drops it: only the upload's failure and the
    # tool's own line are kept.
    U="/Firmware upload failed|^Failed to open/!d; $N; $O"
    logic16_options flashed=0
    emu_case "unplug before the reset hold" --unplug out=0:1 --expect-fail --stderr --filter "$U" -- -d $F -C 0 --samples 8 -O hex
    emu_case "unplug during the upload" --unplug out=0:2 --expect-fail --stderr --filter "$U" -- -d $F -C 0 --samples 8 -O hex
    emu_case "unplug before the release" --unplug out=0:4 --expect-fail --stderr --filter "$U" -- -d $F -C 0 --samples 8 -O hex
}
