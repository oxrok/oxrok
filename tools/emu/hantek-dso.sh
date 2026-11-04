# The Hantek DSO-2090 USB scope (emu/src/models/hantek-dso.rs): an FX2
# without firmware (04b4:2090) that takes the upload and renumerates as
# 04b5:2090, and its siblings DSO-2150, DSO-2250, DSO-5200 and 5200A.
# Sourced by tools/diffcli.sh --emu; see docs/USB.md, "Writing a device
# model".
EMU_MODEL=hantek-dso
EMU_OPTIONS="-o profile=dso-2090"
EMU_DRIVER=hantek-dso
EMU_FULL=hantek-dso
EMU_GCOV_SHARED="src/usb.c src/ezusb.c"

# The model's options for the cases that follow (see fx2lafw.sh).
hantek_options() {
    set -- options "$@"
    emu_cmd "$*" >/dev/null || emu_fail "hantek-dso: the emulator refused '$*'"
}

# What a frame case is held to, in few lines (the analog output has one
# line per sample and channel, 20480 a frame): every 64th line, and,
# through the harness's --digest, a last line with the SHA-256 of the
# whole output, so the filter loses nothing.
HD_FRAME='1~64!d'

# Claims interface 0 of the attached device through usbfs, as another
# program would, until hantek_release (as fx2lafw.sh does).
hantek_hold() {
    [ "$EMU_DRY" = 0 ] || return 0 # diffcli counting cases: no device, nothing to claim
    timeout 120 python3 -I -c '
import fcntl, os, signal, struct, sys
d = "/sys/bus/usb/devices/" + sys.argv[1]
bus, dev = (int(open(d + f).read()) for f in ("/busnum", "/devnum"))
fd = os.open("/dev/bus/usb/%03d/%03d" % (bus, dev), os.O_RDWR)
fcntl.ioctl(fd, 0x8004550f, struct.pack("I", 0))  # USBDEVFS_CLAIMINTERFACE
print("held", flush=True)
signal.pause()
' "$EMU_BUSID" >"$TMP/hantek-hold" 2>&1 &
    HOLDPID=$!
    i=0
    until grep -q '^held$' "$TMP/hantek-hold"; do
        i=$((i + 1))
        if [ $i -gt 100 ] || ! kill -0 $HOLDPID 2>/dev/null; then
            emu_fail "hantek-dso: could not claim the interface: $(cat "$TMP/hantek-hold")"
            return 1
        fi
        sleep 0.1
    done
}

hantek_release() {
    kill $HOLDPID 2>/dev/null
    wait $HOLDPID 2>/dev/null
    HOLDPID=""
}

emu_cases() {
    H=hantek-dso
    # A renumerated device's address, in the text the harness's conn=
    # rule does not reach (--show, --get).
    C='s/conn(=|: )[0-9]+\.[0-9]+/conn\1BUS.ADDR/g'
    # The last stderr line, sigrok-cli's own, made one word: the
    # driver's message before it is in the transactions' wake.
    E='$!d; s/.*/refused/'

    # The DSO-2090 without firmware: every run uploads it in the scan,
    # and opens the renumerated device by its port.
    hantek_options profile=dso-2090
    emu_case "2090 scan" --keep-header -- -d $H --scan
    emu_case "2090 show" --keep-header --filter "$C" -- -d $H --show
    emu_case "2090 by ids" --digest --filter "$HD_FRAME" -- -d $H:conn=04b4.2090 --frames 1
    # The driver's defaults: no trigger source (EXT, nothing on it), no
    # force on the first capture: three empty states, then a restart
    # with a forced trigger.
    emu_case "2090 one frame" --digest --filter "$HD_FRAME" -- -d $H --frames 1
    emu_case "2090 two frames" --digest --filter "$HD_FRAME" -- -d $H --frames 2
    emu_case "2090 CH1 only" --digest --filter "$HD_FRAME" -- -d $H -C CH1 --frames 1
    emu_case "2090 CH2 only" --digest --filter "$HD_FRAME" -- -d $H -C CH2 --frames 1
    emu_case "2090 trigger CH1 rising" --digest --filter "$HD_FRAME" -- -d $H -c triggersource=CH1 --frames 1
    emu_case "2090 trigger CH1 falling ratio 50" --digest --filter "$HD_FRAME" -- -d $H -c triggersource=CH1:triggerslope=f:captureratio=50 --frames 1
    emu_case "2090 trigger CH2 level" --digest --filter "$HD_FRAME" -- -d $H -c triggersource=CH2:triggerlevel=0.4:captureratio=10 --frames 1
    emu_case "2090 trigger CH1 two frames" --digest --filter "$HD_FRAME" -- -d $H -c triggersource=CH1:captureratio=0 --frames 2
    for tb in 10us 20us 40us; do
        emu_case "2090 timebase $tb" --digest --filter "$HD_FRAME" -- -d $H -c timebase=$tb --frames 1
    done
    emu_case "2090 timebase 400ms" --digest --filter "$HD_FRAME" -- -d $H -c timebase=400ms --frames 1
    emu_case "2090 samplerate set" --digest --filter "$HD_FRAME" -- -d $H -c samplerate=1M --frames 1
    emu_case "2090 large frame" --digest --filter "$HD_FRAME" -- -d $H -c buffersize=32768:timebase=1ms --frames 1
    for tb in 40us 100us 200us; do
        emu_case "2090 large frame $tb" --digest --filter "$HD_FRAME" -- -d $H -c buffersize=32768:timebase=$tb --frames 1
    done
    emu_case "2090 large frame 10us" --expect-fail --stderr --filter "$E" -- -d $H -c buffersize=32768:timebase=10us --frames 1
    emu_case "2090 CH1 200mV" --digest --filter "$HD_FRAME" -- -d $H -g CH1 -c vdiv=200mV --frames 1
    emu_case "2090 CH1 10mV" --digest --filter "$HD_FRAME" -- -d $H -g CH1 -c vdiv=10mV --frames 1
    emu_case "2090 CH2 1V" --digest --filter "$HD_FRAME" -- -d $H -g CH2 -c vdiv=1V --frames 1
    emu_case "2090 CH2 2V" --digest --filter "$HD_FRAME" -- -d $H -g CH2 -c vdiv=2V --frames 1
    emu_case "2090 CH2 10mV" --digest --filter "$HD_FRAME" -- -d $H -g CH2 -c vdiv=10mV --frames 1
    emu_case "2090 CH2 5V" --digest --filter "$HD_FRAME" -- -d $H -g CH2 -c vdiv=5V --frames 1
    emu_case "2090 CH1 AC" --digest --filter "$HD_FRAME" -- -d $H -g CH1 -c coupling=AC --frames 1
    emu_case "2090 CH2 GND" --digest --filter "$HD_FRAME" -- -d $H -g CH2 -c coupling=GND --frames 1
    emu_case "2090 CH1 filter" --digest --filter "$HD_FRAME" -- -d $H -g CH1 -c filter=1 --frames 1
    emu_case "2090 CH2 filter" --digest --filter "$HD_FRAME" -- -d $H -g CH2 -c filter=1 --frames 1
    # The gets: what the driver keeps, no transfer after the open.
    for k in timebase samplerate buffersize triggerslope captureratio limit_frames triggerlevel; do
        emu_case "2090 get $k" -- -d $H --get $k
    done
    # config_get() fills in NUM_HDIV and NUM_VDIV, then goes on to the
    # keys of a device and returns SR_ERR_NA for them (api.c:430-483).
    emu_case "2090 get num_hdiv" --expect-fail --stderr --filter "$E" -- -d $H --get num_hdiv
    emu_case "2090 get num_vdiv" --expect-fail --stderr --filter "$E" -- -d $H --get num_vdiv
    emu_case "2090 get conn" --filter "s/^'[0-9]+\.[0-9]+'\$/'BUS.ADDR'/" -- -d $H --get conn
    emu_case "2090 get triggersource set" -- -d $H -c triggersource=CH2 --get triggersource
    emu_case "2090 CH1 show" --keep-header -- -d $H -g CH1 --show
    # The driver's debug messages: the calibration and the relays.
    emu_case "2090 debug log" --digest --filter "$HD_FRAME" -- -l 5 -d $H --frames 1
    # Values not in the driver's lists.
    for kv in triggerslope=x buffersize=1000 timebase=3us samplerate=3M triggersource=CH3; do
        emu_case "2090 set $kv" --expect-fail --stderr --filter "$E" -- -d $H -c $kv --frames 1
    done
    for kv in vdiv=3V coupling=XX; do
        emu_case "2090 set CH1 $kv" --expect-fail --stderr --filter "$E" -- -d $H -g CH1 -c $kv --frames 1
    done
    emu_case "2090 get triggersource" --expect-fail --stderr --filter "$E" -- -d $H --get triggersource
    for k in vdiv coupling filter; do
        emu_case "2090 get CH2 $k" -- -d $H -g CH2 --get $k
    done
    emu_case "2090 trigger level bad" --expect-fail --stderr --filter "$E" -- -d $H -c triggerlevel=1.5 --frames 1

    # A ring position on a transfer boundary (two transfers of 256
    # samples), and none at all.
    hantek_options profile=dso-2090 wrap=512
    emu_case "2090 trigger offset on a transfer boundary" --digest --filter "$HD_FRAME" -- -d $H --frames 1
    # A trigger offset of 0: receive_transfer() sends every chunk as it
    # comes and, at the frame's end, the pre-trigger buffer of 0 samples
    # (api.c:782), an analog packet without data, on which sigrok-cli
    # aborts.
    hantek_options profile=dso-2090 wrap=0 flashed=1
    emu_case "2090 trigger offset 0" --expect-fail --stderr --filter '/double free/!d' -- -d $H --frames 1
    # A longer fill, and a signal on EXT.
    hantek_options profile=dso-2090 fill=4
    emu_case "2090 fill 4" --digest --filter "$HD_FRAME" -- -d $H --frames 1
    hantek_options profile=dso-2090 ext=1
    emu_case "2090 trigger EXT" --digest --filter "$HD_FRAME" -- -d $H -c triggersource=EXT:captureratio=25 --frames 1

    # Already running the firmware: no upload, selection by address.
    hantek_options profile=dso-2090 flashed=1
    emu_case "flashed 2090 scan" --keep-header --filter "$C" -- -d $H --scan
    emu_case "flashed 2090 by ids" --digest --filter "$HD_FRAME" -- -d $H:conn=04b5.2090 --frames 1
    emu_case "flashed 2090 by address" --digest --filter "$HD_FRAME" -- -d $H:conn=$EMU_USB --frames 1
    # Another program holds the interface: the claim fails.
    if hantek_hold; then
        emu_case "interface claimed by another program" --expect-fail --stderr --filter "$E" -- -d $H --frames 1
        hantek_release
    fi
    # On a USB 1.1 port: 64 byte packets, 320 transfers a frame.
    hantek_options profile=dso-2090 speed=full
    emu_case "2090 full speed" --digest --filter "$HD_FRAME" -- -d $H --frames 1

    # The siblings.
    hantek_options profile=dso-2150
    emu_case "2150 scan" --keep-header -- -d $H --scan
    emu_case "2150 one frame" --digest --filter "$HD_FRAME" -- -d $H --frames 1
    hantek_options profile=dso-2250
    emu_case "2250 scan" --keep-header -- -d $H --scan
    emu_case "2250 show" --keep-header --filter "$C" -- -d $H --show
    emu_case "2250 one frame" --digest --filter "$HD_FRAME" -- -d $H --frames 1
    emu_case "2250 trigger CH2 1M" --digest --filter "$HD_FRAME" -- -d $H -C CH2 -c triggersource=CH2:triggerslope=f:samplerate=1M:captureratio=30 --frames 1
    emu_case "2250 trigger CH1 100M" --digest --filter "$HD_FRAME" -- -d $H -C CH1 -c triggersource=CH1:samplerate=100M --frames 1
    # CH2's range on the 2250: the driver's header 0x08 is CH2's bit 3
    # (see the model's voltage()); the 5 V, 500 mV and 50 mV codes read
    # as those of 1 V, 100 mV and 10 mV, so CH2 comes out five times
    # too large there. One case per range relay group and code.
    for vd in 10mV 20mV 50mV 200mV 1V 5V; do
        emu_case "2250 CH2 $vd" --digest --filter "$HD_FRAME" -- -d $H -g CH2 -c vdiv=$vd --frames 1
    done
    emu_case "2250 125M" --expect-fail --stderr --filter "$E" -- -d $H -c samplerate=125M --frames 1
    hantek_options profile=dso-2250 ext=1
    emu_case "2250 trigger EXT" --digest --filter "$HD_FRAME" -- -d $H -c triggersource=EXT --frames 1
    # The DSO-5200s answer a ready frame with state 7, 9 bit samples,
    # which the driver does not read ("Not yet supported.") and polls
    # again for ever: no capture case.
    hantek_options profile=dso-5200
    emu_case "5200 scan" --keep-header -- -d $H --scan
    emu_case "5200 show" --keep-header --filter "$C" -- -d $H --show
    hantek_options profile=dso-5200a
    emu_case "5200A scan" --keep-header -- -d $H --scan
    emu_case "5200A get buffersize" -- -d $H --get buffersize

    # Faults. The upload refused: the board stays a bare FX2, which the
    # driver then does not find under the firmware's ids.
    hantek_options profile=dso-2090 stall=load
    emu_case "fault upload stall" --expect-fail --stderr --filter "$E" -- -d $H --frames 1
    # A chunk of the firmware refused, and the 8051's release.
    for f in load@2 load@4; do
        hantek_options profile=dso-2090 stall=$f
        emu_case "fault upload stall $f" --expect-fail --stderr --filter "$E" -- -d $H --frames 1
    done
    # Firmwares whose endpoints the driver does not know.
    for f in bad swapped one; do
        hantek_options profile=dso-2090 flashed=1 endpoints=$f
        emu_case "fault endpoints $f" --expect-fail --stderr --filter "$E" -- -d $H --frames 1
    done
    # Each step of dso_init() and the capture start failing.
    for f in eeprom relays offset begin@1 begin@2 begin@3 begin@4 begin@5 out@1 out@2 out@3 out@4 out@5; do
        hantek_options profile=dso-2090 flashed=1 stall=$f
        emu_case "fault stall $f" --expect-fail --stderr --filter "$E" -- -d $H --frames 1
    done
    # The trigger level's offsets refused while setting it.
    hantek_options profile=dso-2090 flashed=1 stall=offset
    emu_case "fault stall offset on set" --expect-fail --stderr --filter "$E" -- -d $H -c triggerlevel=0.3 --frames 1
    # The samplerate's trigger command refused while setting it, on the
    # 2090 (the begin) and the 2250 (each of its first commands).
    hantek_options profile=dso-2090 flashed=1 stall=out@1
    emu_case "fault samplerate set stall" --expect-fail --stderr --filter "$E" -- -d $H -c samplerate=1M --frames 1
    for f in begin@1 begin@2 begin@3 begin@4 begin@5 out@1 out@2 out@3 out@4 out@5; do
        hantek_options profile=dso-2250 flashed=1 stall=$f
        emu_case "fault 2250 stall $f" --expect-fail --stderr --filter "$E" -- -d $H --frames 1
    done
    # Faults during the capture, which the driver rides out: a failed
    # poll (its begin, its command, its answer) is polled again, as are
    # a timeout state, an unknown one and a 9 bit ready.
    for f in stall=begin@7 stall=out@7 stall=state@2 state=127@2 state=255@2 state=7@3; do
        hantek_options profile=dso-2090 flashed=1 $f
        emu_case "fault capture $f" --digest --filter "$HD_FRAME" -- -d $H --frames 1
    done
    # Faults no capture recovers from: the driver ends on none of them,
    # and sigrok-cli runs until the harness stops it (--hangs). A
    # channel data transfer that completes empty leaves the frame one
    # packet short, and the driver waits for the transfer it never
    # submitted; no transfer after.
    hantek_options profile=dso-2090 flashed=1 empty=3
    emu_case "fault empty data transfer hangs" --hangs --timeout 5 --stderr --no-output -- -d $H --frames 1
    # Unplugged in the middle of the frame (the 20th IN of endpoint 6:
    # six state polls, then the data): the rest of the transfers fail
    # empty, as above. Unplugged while it polls (the 3rd): every poll
    # fails from then on and is tried again each millisecond.
    hantek_options profile=dso-2090 flashed=1
    emu_case "unplug mid frame hangs" --hangs --timeout 5 --stderr --no-output --unplug in=6:20 -- -d $H --frames 1
    emu_case "unplug while polling hangs" --hangs --timeout 5 --stderr --unplug in=6:3 -- -d $H --frames 1
    # A trigger that never fires (EXT with nothing on it): the driver
    # polls, restarts the capture without a force every third empty
    # state, and polls on. The transactions are cut after one period.
    emu_case "trigger never fires hangs" --hangs --timeout 5 --stderr --no-output -- -d $H -c triggersource=EXT --frames 1
    # A DSO-5200's ready frame (state 7, 9 bit samples): "Not yet
    # supported." at each poll, for ever.
    hantek_options profile=dso-5200 flashed=1
    emu_case "5200 capture hangs" --hangs --timeout 5 --stderr -- -d $H --frames 1
    # The begin or a command of a capture restart refused: the next poll
    # finds the device still empty, three times, and restarts again.
    # (The begins: 1 to 5 dso_init() and the start, 6 to 9 the polls,
    # 10 to 12 the restart's start, enable and force, 13 and 14 polls,
    # 15 the channel data.)
    for f in begin@10 begin@11 begin@12 out@10 out@11 out@12; do
        hantek_options profile=dso-2090 flashed=1 stall=$f
        emu_case "fault restart stall $f" --digest --filter "$HD_FRAME" -- -d $H --frames 1
    done
    # The channel data request refused: polled again, still ready.
    for f in begin@15 out@15; do
        hantek_options profile=dso-2090 flashed=1 stall=$f
        emu_case "fault channel data stall $f" --digest --filter "$HD_FRAME" -- -d $H --frames 1
    done
    # The second frame's start, enable and force refused: tried again.
    for f in begin@16 begin@17 begin@18; do
        hantek_options profile=dso-2090 flashed=1 stall=$f
        emu_case "fault next frame stall $f" --digest --filter "$HD_FRAME" -- -d $H --frames 2
    done
    hantek_options
}
