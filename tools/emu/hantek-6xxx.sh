# The Hantek 6022BE/BL scopes and their relatives (emu/src/models/hantek-6xxx.rs):
# an FX2 that takes the sigrok scope firmware and renumerates as
# 1d50:608e, two analog channels of 8 bit samples, the input range per
# channel, the rate and, on the DDS120 and ISDS205B, the coupling set by
# vendor requests. Sourced by tools/diffcli.sh --emu; see docs/USB.md,
# "Writing a device model". Every case runs both tools; see the --oracle-*
# cases for what oxrok does not copy.
EMU_MODEL=hantek-6xxx
EMU_OPTIONS="-o profile=6022be -o flashed=1"
EMU_DRIVER=hantek-6xxx
EMU_FULL=hantek-6xxx
EMU_GCOV_SHARED="src/usb.c src/ezusb.c"

# The model's options for the cases that follow (docs/USB.md, `options`).
hantek_options() {
    set -- options "$@"
    emu_cmd "$*" >/dev/null || emu_fail "hantek-6xxx: the emulator refused '$*'"
}

# Claims interface 0 of the attached device through usbfs, as another
# program would, and holds it until hantek_release (at most 120 s). The
# device must not be re-attached meanwhile. A claim is no transfer, so
# the model sees nothing of it. (As fx2lafw_hold in tools/emu/fx2lafw.sh.)
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
            emu_fail "hantek-6xxx: could not claim the interface: $(cat "$TMP/hantek-hold")"
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
    H=hantek-6xxx
    # A renumerated device's address, in the text the harness's conn=
    # rule does not reach (--show, --get).
    C='s/conn(=|: )[0-9]+\.[0-9]+/conn\1BUS.ADDR/g'
    A="s/'[0-9]+\\.[0-9]+'/'BUS.ADDR'/"
    # The last line of stderr, for a failure both tools word their own way.
    L='$!d'
    # The tool's own line when the open fails; oxrok adds the reason.
    O='$!d; s/^(Failed to open device).*$/\1./'
    # A refused option: oxrok says why after "invalid argument", and "not
    # supported" for a key the device or group lacks, sigrok-cli's "not
    # applicable". Only the wording goes: the option named stays, and which
    # of the two refusals it was.
    R='$!d; s/(invalid argument).*$/\1./; s/: not supported\.$/: not applicable./'

    # EMU_USB is the address of the first attach, the flashed 6022BE of
    # EMU_OPTIONS: first, before other descriptors re-attach the device.
    hantek_options profile=6022be flashed=1
    emu_case "flashed 6022be by address" -- -d "$H:conn=$EMU_USB" -c samplerate=1M --samples 8 -O analog

    # The 6022BE without firmware: every run uploads it in the scan,
    # and the scope renumerates before it is opened. Each capture first
    # reads 1024 bytes the driver throws away (its "flush"), then one
    # transfer of the samples asked for, rounded up to a power of two of
    # at least 512 bytes, all of which the driver sends on.
    hantek_options profile=6022be
    emu_case "6022be scan" --keep-header -- -d $H --scan
    emu_case "6022be show" --keep-header --filter "$C" -- -d $H --show
    emu_case "6022be show ch1" --keep-header -- -d $H -g CH1 --show
    emu_case "6022be by ids" -- -d $H:conn=04b4.6022 --samples 8 -O analog
    emu_case "6022be default 8M" -- -d $H --samples 100 -O csv:header=false
    emu_case "6022be 100k" -- -d $H -c samplerate=100k --samples 200 -O analog
    emu_case "6022be 200k" -- -d $H -c samplerate=200k --samples 300 -O csv:header=false
    emu_case "6022be 500k" -- -d $H -c samplerate=500k --samples 256 -O csv:header=false
    emu_case "6022be 1M" -- -d $H -c samplerate=1M --samples 1000 -O csv:header=false
    emu_case "6022be 4M" -- -d $H -c samplerate=4M --samples 2 -O csv:header=false
    emu_case "6022be 16M" -- -d $H -c samplerate=16M --samples 20 -O csv:header=false
    emu_case "6022be 24M" -- -d $H -c samplerate=24M --samples 20 -O csv:header=false
    emu_case "6022be 30M" -- -d $H -c samplerate=30M --samples 20 -O csv:header=false
    emu_case "6022be 48M" -- -d $H -c samplerate=48M --samples 20 -O csv:header=false
    # 4096 transfers' worth: one transfer of 256 KiB after the flush.
    emu_case "6022be 100k 100000 samples" --filter '1~997!d' -- -d $H -c samplerate=100k --samples 100000 -O csv:header=false
    # The range of each channel: 1 V/div clips nothing, 100 mV/div clips both.
    emu_case "6022be ch1 1V" -- -d $H -g CH1 -c vdiv=1V --samples 100 -O analog
    emu_case "6022be ch2 500mV" -- -d $H -g CH2 -c vdiv=500mV --samples 100 -O analog
    emu_case "6022be ch1 100mV" -- -d $H -g CH1 -c vdiv=100mV --samples 100 -O analog
    emu_case "6022be ch2 100mV" -- -d $H -g CH2 -c vdiv=100mV --samples 100 -O analog
    emu_case "6022be bad vdiv" --expect-fail --stderr --filter "$R" -- -d $H -g CH1 -c vdiv=2V --samples 8 -O analog
    # One channel: the device sends both, the driver sends the enabled one.
    emu_case "6022be ch1 only" -- -d $H -C CH1 -c samplerate=100k --samples 100 -O analog
    # With CH1 disabled, send_chunk() still takes the first channel of
    # enabled_channels (configure_channels() puts every channel in it),
    # so CH2's samples come out labelled CH1.
    emu_case "6022be ch2 only" --oracle-differs "api.c:187-192, 588: CH2's samples are labelled CH1" -- -d $H -C CH2 -c samplerate=100k --samples 100 -O analog
    # A rate outside the list: the driver keeps it, logs that it cannot
    # convert it and sends the last register, 100 kHz (protocol.c:152-162).
    # The same goes out here and the samples are the same, but the C
    # driver goes on saying 2 MHz over samples taken at 100 kHz.
    emu_case "6022be unlisted 2M" -- -d $H -c samplerate=2M --samples 100 -O analog
    emu_case "6022be unlisted 2M reads back" --oracle-differs "api.c:441, 388: the rate asked for, where protocol.c sends 100 kHz" -- -d $H -c samplerate=2M --get samplerate
    emu_case "6022be unlisted 2M times" --oracle-differs "api.c:441: csv's time column at 2 MHz over samples taken at 100 kHz" -- -d $H -c samplerate=2M --samples 4 -O csv:header=false:time=true
    # A time limit alone has no case: receive_transfer() ends the capture
    # once g_get_monotonic_time() says limit_msec went by, so how many
    # transfers it takes depends on wall time.
    # A time limit with a sample limit: data_amount() sizes the
    # transfer from the time left (10 ms at 100 kHz, 2000 bytes, 2048
    # whether 0 or 4 ms have gone), and the sample limit ends the capture.
    emu_case "6022be time and samples" -- -d $H -c samplerate=100k --time 10 --samples 300 -O csv:header=false
    emu_case "6022be get samplerate" -- -d $H --get samplerate
    emu_case "6022be get limit_samples" -- -d $H --get limit_samples
    emu_case "6022be get limit_time" -- -d $H --get limit_time
    emu_case "6022be get ch1 vdiv" -- -d $H -g CH1 --get vdiv
    emu_case "6022be get ch2 coupling" -- -d $H -g CH2 --get coupling
    # For the device as a whole, config_get() answers NUM_VDIV in its
    # first switch, then its second one says SR_ERR_NA. On a channel
    # group libsigrok's check_key() (hwdriver.c:768-780) refuses before
    # the driver is asked: NUM_VDIV is not in devopts_cg.
    emu_case "6022be get num_vdiv" --oracle-bug "api.c:380-383, 406-407: the answer is set, then the second switch says SR_ERR_NA" -- -d $H --get num_vdiv
    emu_case "6022be get ch1 num_vdiv" --expect-fail --stderr --filter "$L" -- -d $H -g CH1 --get num_vdiv
    # The address the scope renumerated to, known once it is open.
    emu_case "6022be get conn" --filter "$A" -- -d $H --get conn
    # --set writes the range of both channels on the open device.
    emu_case "6022be set ch2 vdiv" --no-output -- -d $H -g CH2 --set -c vdiv=100mV
    emu_case "6022be set coupling dc" --no-output -- -d $H -g CH1 --set -c coupling=DC
    # coupling=DC is index 0 of the 6022BE's one-entry list, stored as
    # the coupling, and 0 is COUPLING_AC: the driver then reports AC.
    emu_case "6022be coupling dc reads back ac" --oracle-differs "api.c:468-471, 422-423: index 0 of the one-entry list is stored as the coupling, and 0 is COUPLING_AC" -- -d $H -g CH1 -c coupling=DC --get coupling
    emu_case "6022be set coupling ac" --expect-fail --stderr --filter "$R" -- -d $H -g CH1 --set -c coupling=AC
    # A conn that names nothing: no device, and sr_usb_find()'s message.
    N='s/^sr: [a-z0-9-]+: //; s/^oxrok: [a-z]+: [a-z0-9_:]+: //'
    emu_case "6022be bad conn" --stderr --filter "$N" -- -d $H:conn=nonsense --scan
    # Options the driver refuses for the device as a whole, or for a
    # channel group.
    emu_case "6022be set vdiv without group" --expect-fail --stderr --filter "$R" -- -d $H --set -c vdiv=1V
    emu_case "6022be set samplerate on group" --expect-fail --stderr --filter "$R" -- -d $H -g CH1 --set -c samplerate=1M

    # The 6022BE as the other Windows driver leaves it, and the
    # Voltcraft DSO-2020, which takes the 6022BE's firmware.
    hantek_options profile=6022be-04b5
    emu_case "6022be 04b5 scan" --keep-header -- -d $H --scan
    emu_case "6022be 04b5 1M" -- -d $H -c samplerate=1M --samples 20 -O csv:header=false
    hantek_options profile=dso2020
    emu_case "dso2020 scan" --keep-header -- -d $H --scan
    emu_case "dso2020 500k" -- -d $H -c samplerate=500k --samples 20 -O csv:header=false

    # The Sainsmart DDS120: AC and DC coupling (0xe5).
    hantek_options profile=dds120
    emu_case "dds120 scan" --keep-header -- -d $H --scan
    emu_case "dds120 show ch1" --keep-header -- -d $H -g CH1 --show
    emu_case "dds120 dc" -- -d $H -c samplerate=100k --samples 100 -O analog
    emu_case "dds120 ch1 ac" -- -d $H -g CH1 -c coupling=AC --samples 100 -O analog
    # sigrok-cli sets -c's options in its hash table's order, vdiv first
    # whichever is written first, oxrok in the order given.
    emu_case "dds120 ch2 ac 1V" -- -d $H -g CH2 -c vdiv=1V:coupling=AC --samples 100 -O analog
    emu_case "dds120 set coupling ac" --no-output -- -d $H -g CH2 --set -c coupling=AC
    emu_case "dds120 get ch1 coupling" -- -d $H -g CH1 --get coupling

    # The 6022BL, both ids.
    hantek_options profile=6022bl
    emu_case "6022bl scan" --keep-header -- -d $H --scan
    emu_case "6022bl 8M" -- -d $H --samples 30 -O csv:header=false
    hantek_options profile=6022bl-04b5
    emu_case "6022bl 04b5 scan" --keep-header -- -d $H --scan

    # The YiXingDianZi MDSO.
    hantek_options profile=mdso
    emu_case "mdso scan" --keep-header -- -d $H --scan
    emu_case "mdso ch1 500mV" -- -d $H -g CH1 -c vdiv=500mV --samples 50 -O analog

    # The Instrustar ISDS205B: six ranges of its own, and coupling.
    hantek_options profile=isds205b
    emu_case "isds205b scan" --keep-header -- -d $H --scan
    emu_case "isds205b show ch2" --keep-header -- -d $H -g CH2 --show
    emu_case "isds205b ch1 705mV" -- -d $H -g CH1 -c vdiv=705mV --samples 100 -O analog
    emu_case "isds205b ch2 140mV ac" -- -d $H -g CH2 -c vdiv=140mV:coupling=AC --samples 100 -O analog
    # sigrok-cli parses a voltage as a whole number of V or mV, and the
    # driver matches the fraction exactly: 1.28 V, 57.6 mV and 17.6 mV
    # (128/100, 576/10000, 176/10000) cannot be set from it. oxrok's
    # parser takes the fractions, so the three ranges work there.
    emu_case "isds205b ch1 1.28V" --oracle-bug "sigrok-cli's sr_parse_voltage() takes whole V or mV only" -- -d $H -g CH1 -c vdiv=1.28V --samples 100 -O analog
    emu_case "isds205b ch1 57.6mV" --oracle-bug "sigrok-cli's sr_parse_voltage() takes whole V or mV only" -- -d $H -g CH1 -c vdiv=57.6mV --samples 100 -O analog

    # The firmware already runs: no upload, and the driver knows the
    # address from the scan.
    hantek_options profile=6022be flashed=1
    emu_case "flashed 6022be scan" --keep-header --filter "$C" -- -d $H --scan
    emu_case "flashed 6022be show" --keep-header --filter "$C" -- -d $H --show
    emu_case "flashed 6022be get conn" --filter "$A" -- -d $H --get conn
    emu_case "flashed 6022be by ids" --record -- -d $H:conn=1d50.608e --samples 8 -O analog
    emu_case "flashed 6022be 4M" -- -d $H -c samplerate=4M --samples 100 -O csv:header=false
    hantek_options profile=dds120 flashed=1
    emu_case "flashed dds120 scan" --keep-header --filter "$C" -- -d $H --scan
    emu_case "flashed dds120 ac" --record -- -d $H -g CH1 -c coupling=AC --samples 50 -O analog
    hantek_options profile=6022bl flashed=1
    emu_case "flashed 6022bl scan" --keep-header --filter "$C" -- -d $H --scan
    hantek_options profile=mdso flashed=1
    emu_case "flashed mdso scan" --keep-header --filter "$C" -- -d $H --scan
    hantek_options profile=isds205b flashed=1
    emu_case "flashed isds205b scan" --keep-header --filter "$C" -- -d $H --scan

    # Faults. A STALLed configuration request is logged and the capture
    # goes on with the scope's old setting (1 MHz after the firmware's
    # init, while the driver labels the data with 100 kHz).
    hantek_options profile=6022be flashed=1 stall=samplerate
    emu_case "fault samplerate stall" -- -d $H -c samplerate=100k --samples 100 -O analog
    hantek_options profile=6022be flashed=1 stall=vdiv1,vdiv2
    emu_case "fault vdiv stall" -- -d $H -c samplerate=100k --samples 100 -O analog
    hantek_options profile=dds120 flashed=1 stall=coupling
    emu_case "fault coupling stall" -- -d $H -g CH1 -c coupling=AC --samples 50 -O analog
    # The stop STALLed: logged, the scope goes on sampling, the run ends.
    hantek_options profile=6022be flashed=1 stall=trigger/0
    emu_case "fault stop stall" -- -d $H -c samplerate=100k --samples 50 -O analog
    # The flush transfer empty or STALLed: the driver ignores what it
    # holds anyway. A second one ends the capture for good: the driver
    # (receive_transfer()) neither resubmits nor stops, and sigrok-cli
    # waits until the harness stops it, with no transfer after. oxrok
    # takes the next transfer, and gives up after 64 in a row.
    hantek_options profile=6022be flashed=1 empty=2
    emu_case "fault empty after the flush hangs" --oracle-bug "api.c:647-649 returns without the next transfer" --oracle-hangs --timeout 5 -- -d $H -c samplerate=100k --samples 50 -O analog
    hantek_options profile=6022be flashed=1 stall_in=2
    emu_case "fault stalled after the flush hangs" --oracle-bug "api.c:647-649 returns without the next transfer" --oracle-hangs --timeout 5 -- -d $H -c samplerate=100k --samples 50 -O analog
    hantek_options profile=6022be flashed=1 empty=1000
    emu_case "fault always empty" --oracle-bug "api.c:647-649 returns without the next transfer" --oracle-hangs --timeout 5 --stderr --filter "$L" -- -d $H -c samplerate=100k --samples 50 -O analog
    hantek_options profile=6022be flashed=1 stall_in=1000
    emu_case "fault always stalled" --oracle-bug "api.c:647-649 returns without the next transfer" --oracle-hangs --timeout 5 --stderr --filter "$L" -- -d $H -c samplerate=100k --samples 50 -O analog
    # The start STALLed: logged, and the driver reads all the same. The
    # scope does not sample: the flush runs out after 4 s, and so does
    # the next transfer, on which the C driver stops as above, oxrok ends.
    hantek_options profile=6022be flashed=1 stall=trigger/1
    emu_case "fault start stall" --oracle-bug "api.c:753 ignores the failed start, then api.c:647-649" --oracle-hangs --timeout 15 --stderr --filter "$L" -- -d $H -c samplerate=100k --samples 50 -O analog
    # A transfer that outlasts the timeout: both take what it held, the
    # flush after 4 s, the samples after 4 more. The next one is not
    # needed: 256 samples are past the limit.
    hantek_options profile=6022be flashed=1 slow=0
    emu_case "fault slow transfers" --timeout 30 -- -d $H -c samplerate=100k --samples 50 -O analog
    hantek_options profile=6022be flashed=1 empty=1
    emu_case "fault empty flush" -- -d $H -c samplerate=100k --samples 50 -O analog
    hantek_options profile=6022be flashed=1 stall_in=1
    emu_case "fault stalled flush" -- -d $H -c samplerate=100k --samples 50 -O analog
    # Short packets: the driver asks again for what is missing.
    hantek_options profile=6022be flashed=1 short=200
    emu_case "fault short transfers" -- -d $H -c samplerate=100k --samples 300 -O csv:header=false
    # A firmware of another board (bcdDevice 9): the scan does not know
    # the flashed scope; after an upload the driver opens it by its ids
    # alone.
    hantek_options profile=6022be flashed=1 bcd=0009
    emu_case "fault flashed other firmware" --no-output -- -d $H --scan
    # oxrok warns of it: nothing shows that such a firmware cannot work
    # (another version of the file may say otherwise), so it is no refusal.
    hantek_options profile=6022be bcd=0009
    emu_case "fault other firmware after upload" -- -d $H -c samplerate=100k --samples 8 -O analog
    # The upload fails: the CPUCS hold, a chunk of code or the release
    # STALLed. The scope stays an FX2 without firmware, and the open
    # finds no 1d50:608e.
    hantek_options profile=6022be stall=hold
    emu_case "fault upload hold stall" --expect-fail --stderr --filter "$O" -- -d $H --samples 8 -O analog
    hantek_options profile=6022be stall=chunk
    emu_case "fault upload chunk stall" --expect-fail --stderr --filter "$O" -- -d $H --samples 8 -O analog
    hantek_options profile=6022be stall=release
    emu_case "fault upload release stall" --expect-fail --stderr --filter "$O" -- -d $H --samples 8 -O analog
    # Another program holds the interface: the claim fails (busy). The
    # same descriptors as the case before, so no re-attach.
    hantek_options profile=6022be flashed=1
    emu_case "flashed 6022be before the claim" -- -d $H --samples 4 -O analog
    if hantek_hold; then
        emu_case "interface claimed by another program" --expect-fail --stderr --filter "$O" -- -d $H --samples 8 -O analog
        hantek_release
    fi
    # Unplugged once the samples are in: the stop request fails, logged,
    # and the run ends with what it got.
    hantek_options profile=6022be flashed=1
    emu_case "unplug after the samples" --unplug in=6:2 -- -d $H -c samplerate=100k --samples 50 -O analog
    # Unplugged with the first transfer of samples in flight: it fails,
    # no next one, and the C driver waits for ever; oxrok ends.
    hantek_options profile=6022be flashed=1
    emu_case "unplug during the capture" --unplug in=6:1 --oracle-bug "api.c:647-649 returns without the next transfer" --oracle-hangs --timeout 5 --stderr --filter "$L" -- -d $H -c samplerate=100k --samples 50 -O analog
    # A limit past 2^31 bytes: data_amount() rounds in 32 bits, wraps to
    # 0 and never ends (api.c:540-541), spinning right after the flush.
    # oxrok asks for 12 MiB transfers; the unplug after the flush keeps
    # the case short. It costs the case its edge: an unplug at that point
    # hangs the C driver anyway (the case before), so what pins the size
    # is this .tx and huge_limits_take_the_maximum, not the hang.
    hantek_options profile=6022be flashed=1
    emu_case "limit past 2G bytes" --unplug in=6:1 --oracle-bug "api.c:540-541: the uint32_t that rounds up wraps to 0" --oracle-hangs --timeout 5 --stderr --filter "$L" -- -d $H -c samplerate=100k --samples 1500000000 -O analog
    # Ctrl-C. Three transfers: the flush, two at 200 bytes each; the next
    # is held, so the count is known, and comes when its timeout runs out.
    hantek_options profile=6022be flashed=1 short=200 slow=3
    emu_case "interrupt during a capture" --interrupt in=6:3 --oracle-bug "sigrok-cli has no SIGINT handler" --timeout 30 -- -d $H -c samplerate=100k --samples 100000 -O csv:header=false
    hantek_options
}
