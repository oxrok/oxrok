# The Kingst LA series (emu/src/models/kingst-la2016/mod.rs): the FX2
# without firmware that takes kingst-la-01a2.fw and renumerates, the
# EEPROM that names the model, the FPGA bitstream, the captures to the
# sample memory with the hardware trigger and their run length encoded
# download, the streaming mode, the PWM outputs and the threshold.
# Sourced by tools/diffcli.sh --emu; see docs/USB.md, "Writing a device
# model".
EMU_MODEL=kingst-la2016
EMU_OPTIONS="-o mcu=1"
EMU_DRIVER=kingst-la2016
EMU_GCOV_SHARED="src/usb.c src/ezusb.c"
# oxrok has the driver: its cases must run full.
EMU_FULL=kingst-la2016

# The model's options for the cases that follow: every case's reset
# builds the model with them, and re-attaches the device when its
# descriptors change.
kingst_options() {
    # The emulator trims what it acknowledges: no trailing space.
    set -- options "$@"
    emu_cmd "$*" >/dev/null || emu_fail "kingst-la2016: the emulator refused '$*'"
}

# Claims interface 0 of the attached device through usbfs, as another
# program would, and holds it until kingst_release (at most 120 s, so
# an interrupted run leaves nothing behind). The device must not be
# re-attached meanwhile: the cases between keep its descriptors. A claim
# is no transfer, so the model sees nothing of it.
kingst_hold() {
    [ "$EMU_DRY" = 0 ] || return 0 # diffcli counting cases: no device, nothing to claim
    timeout 120 python3 -I -c '
import fcntl, os, signal, struct, sys
d = "/sys/bus/usb/devices/" + sys.argv[1]
bus, dev = (int(open(d + f).read()) for f in ("/busnum", "/devnum"))
fd = os.open("/dev/bus/usb/%03d/%03d" % (bus, dev), os.O_RDWR)
fcntl.ioctl(fd, 0x8004550f, struct.pack("I", 0))  # USBDEVFS_CLAIMINTERFACE
print("held", flush=True)
signal.pause()
' "$EMU_BUSID" >"$TMP/kingst-hold" 2>&1 &
    HOLDPID=$!
    i=0
    until grep -q '^held$' "$TMP/kingst-hold"; do
        i=$((i + 1))
        if [ $i -gt 100 ] || ! kill -0 $HOLDPID 2>/dev/null; then
            emu_fail "kingst-la2016: could not claim the interface: $(cat "$TMP/kingst-hold")"
            return 1
        fi
        sleep 0.1
    done
}

kingst_release() {
    kill $HOLDPID 2>/dev/null
    wait $HOLDPID 2>/dev/null
    HOLDPID=""
}

emu_cases() {
    K=kingst-la2016
    # A device that renumerated has an address the READY line did not
    # report.
    C='s/conn(=|: )[0-9]+\.[0-9]+/conn\1BUS.ADDR/g'
    # oxrok's log prefix ("oxrok: LEVEL: MODULE: ") as libsigrok's, so
    # that the filters below read one form.
    N='s/^oxrok: [a-z]+: oxrok::drivers::kingst_la2016[a-z0-9_:]*: /sr: kingst-la2016: /; s/^oxrok: [a-z]+: oxrok::ezusb: /sr: ezusb: /'
    # The last line of stderr: the tool's own words for the failure, with
    # sr_strerror()'s wording for oxrok's errors.
    L='$!d; s/(invalid argument).*$/\1./; s/: (not supported|unknown option)[^.]*(\.?)$/: not applicable\2/'
    # Only the log lines that say what failed (the driver's sr_err and
    # sr_warn, not libusb's or the tool's).
    E="$N; /kingst-la2016: /!d; s/^.*kingst-la2016: //"

    # The firmware runs (EMU_OPTIONS): no upload, the FPGA is blank
    # and gets its bitstream in every scan. First, while the device is
    # still on the address the READY line reported, the selection by it.
    emu_case "by address" --keep-header -- -d $K:conn=$EMU_USB --scan
    emu_case "scan" --keep-header --filter "$C" -- -d $K --scan
    emu_case "show" --keep-header --filter "$C" -- -d $K --show
    emu_case "by other ids" --stderr --filter "$E" -- -d $K:conn=77a1.01a3 --scan
    # sr_usb_find()'s own message differs in its prefix: the driver's alone.
    emu_case "bad conn" --stderr --filter "$N; /Cannot find/!d; s/^.*kingst-la2016: //" -- -d $K:conn=nonsense --scan
    emu_case "probe names" --keep-header -- -d $K:probe_names=a,b,c,d --scan
    emu_case "get samplerate" -- -d $K --get samplerate
    emu_case "get limit_samples" -- -d $K --get limit_samples
    emu_case "get captureratio" -- -d $K --get captureratio
    emu_case "get voltage_threshold" -- -d $K --get voltage_threshold
    emu_case "get continuous" -- -d $K --get continuous
    # Another program holds the interface: the scan's identification
    # cannot claim it (LIBUSB_ERROR_BUSY) and finds nothing. The same
    # descriptors as the cases before, so the device is not re-attached.
    if kingst_hold; then
        emu_case "interface claimed by another program" --stderr --filter "$E" -- -d $K --scan
        kingst_release
    fi
    emu_case "get conn" --filter "s/^'[0-9]+\\.[0-9]+'\$/'BUS.ADDR'/" -- -d $K --get conn
    emu_case "set threshold" --no-output -- -d $K --set -c voltage_threshold=2.5-2.5
    emu_case "set bad threshold" --expect-fail --stderr --filter "$L" -- -d $K --set -c voltage_threshold=3.3-3.3
    emu_case "get limit_frames" --expect-fail --stderr --filter "$L" -- -d $K --get limit_frames
    # The Logic channel group has no option of its own (the threshold
    # is the device's, WITH_THRESHOLD_DEVCFG).
    emu_case "logic group show" --keep-header --filter "$C" -- -d $K -g Logic --show
    emu_case "logic group get" --expect-fail --stderr --filter "$L" -- -d $K -g Logic --get voltage_threshold
    emu_case "logic group set" --expect-fail --stderr --filter "$L" -- -d $K -g Logic --set -c samplerate=1M
    # The PWM outputs: their channel groups, set and get.
    emu_case "pwm1 get samplerate" --expect-fail --stderr --filter "$L" -- -d $K -g PWM1 --get samplerate
    emu_case "pwm1 show" --keep-header -- -d $K -g PWM1 --show
    emu_case "pwm1 get frequency" -- -d $K -g PWM1 --get output_frequency
    emu_case "pwm2 get duty" -- -d $K -g PWM2 --get output_duty_cycle
    emu_case "pwm2 get enabled" -- -d $K -g PWM2 --get enabled
    emu_case "pwm1 enable" --no-output -- -d $K -g PWM1 --set -c enabled=on
    # sigrok-cli keeps the pairs of one -c in a GHashTable and sets them in
    # its order, g_hash_table_iter_next() at session.c:700-701 (frequency,
    # enabled, duty); oxrok sets them as given. The device ends up alike;
    # the steps differ.
    emu_case "pwm2 frequency and duty" --oracle-differs "sigrok-cli sets the pairs of -c in its hash table order (session.c:700-701), oxrok as given" --no-output -- -d $K -g PWM2 --set -c output_frequency=12345:output_duty_cycle=12.5:enabled=on
    emu_case "pwm1 20 MHz" --oracle-differs "sigrok-cli sets the pairs of -c in its hash table order (session.c:700-701), oxrok as given" --no-output -- -d $K -g PWM1 --set -c enabled=on:output_frequency=20000000:output_duty_cycle=50
    emu_case "pwm1 too fast" --expect-fail --stderr --filter "$L" -- -d $K -g PWM1 --set -c output_frequency=30000000
    emu_case "pwm2 bad duty" --expect-fail --stderr --filter "$L" -- -d $K -g PWM2 --set -c output_duty_cycle=101
    emu_case "pwm2 zero duty" --expect-fail --stderr --filter "$L" -- -d $K -g PWM2 --set -c output_duty_cycle=0

    # Captures to the sample memory, at a few rates and sizes.
    emu_case "capture 20k" -- -d $K -c samplerate=20k --samples 64 -O hex
    emu_case "capture 1M" -- -d $K -c samplerate=1M --samples 1000 -O hex
    emu_case "capture 200M" -- -d $K -c samplerate=200M --samples 2000 -O hex
    emu_case "capture one sample" -- -d $K --samples 1 -O bits
    emu_case "capture five samples" -- -d $K --samples 5 -O bits
    emu_case "capture long runs" -- -d $K -C CH0,CH8 -c samplerate=10M --samples 3000 -O bits:width=0
    emu_case "capture two channels" -- -d $K -C CH1,CH9 --samples 64 -O bits
    # A time limit: sigrok-cli passes it as is, the driver sends "no
    # sample limit" (10 G) and halts the capture at the first poll that
    # sees the time passed. The model samples 50 ms per poll (1000
    # samples at 20 kHz), so the 5 ms are over at the first poll.
    emu_case "capture time" -- -d $K -c samplerate=20k --time 5 -O hex
    # The same while a trigger on CH15 waits (it fires after 128000
    # samples): the memory holds what came before the trigger point.
    emu_case "capture time waiting for the trigger" -- -d $K -c samplerate=20k --time 5 -t CH15=1 -O hex
    # Rates off the list are the driver's too: it sends clock / rate.
    emu_case "capture 3M" -- -d $K -c samplerate=3M --samples 40 -O hex
    # The lowest rate the driver takes, clock / 65536 = 3051 Hz: its
    # divider, 65552, is truncated to 16 bits (16, 12.5 MHz), at
    # protocol.c:757 "divider_u16 = baseclock / devc->samplerate;". oxrok
    # refuses the rate. sigrok-cli's stdout is what the .oracle keeps:
    # the first channel's samples.
    emu_case "capture lowest rate" --oracle-differs "protocol.c:757 truncates the divider to 16 bits" --expect-fail --stderr --filter "$N; /^CH0:|kingst-la2016: Too low/!d; s/^.*kingst-la2016: //" -- -l 5 -d $K -c samplerate=3051 --samples 8 -O hex
    # Two bulk IN transfers of 512 KiB: about 1.6 M samples each.
    emu_case "capture two transfers" --filter '1,/^CH15/!d' -- -d $K -c samplerate=100M --samples 2000000 -O hex
    emu_case "capture too fast" --expect-fail --stderr --filter "$E" -- -d $K -c samplerate=500M --samples 8 -O hex
    emu_case "capture too slow" --expect-fail --stderr --filter "$E" -- -d $K -c samplerate=1k --samples 8 -O hex
    emu_case "capture no logic channel" --expect-fail --stderr --filter "$E" -- -d $K -C PWM1 --samples 8 -O hex
    emu_case "capture threshold 4.0" -- -d $K -c voltage_threshold=4.0-4.0 --samples 8 -O hex
    emu_case "capture threshold 0.4" -- -d $K -c voltage_threshold=0.4-0.4 --samples 8 -O hex
    emu_case "capture verbose" --filter "$C" -- -l 5 -d $K --samples 8 -O hex

    # The hardware trigger: the trigger point is where the sample
    # memory's packets before it end.
    emu_case "trigger rising" -- -d $K -c samplerate=1M --samples 100 -t CH0=r -O bits
    emu_case "trigger falling" -- -d $K -c samplerate=1M --samples 100 -t CH2=f -O bits
    emu_case "trigger high" -- -d $K --samples 40 -t CH3=1 -O bits
    emu_case "trigger low and rising" -- -d $K --samples 40 -t CH1=0,CH0=r -O bits
    emu_case "trigger levels" -- -d $K --samples 40 -t CH0=1,CH1=1,CH2=0 -O bits
    emu_case "trigger ratio 0" -- -d $K -c captureratio=0 --samples 40 -t CH0=f -O bits
    emu_case "trigger ratio 90" -- -d $K -c captureratio=90 --samples 40 -t CH0=f -O bits
    emu_case "trigger ratio 100" -- -d $K -c captureratio=100 --samples 40 -t CH1=r -O bits
    emu_case "trigger high channel" -- -d $K -c samplerate=50M --samples 2000 -t CH8=r -O hex
    emu_case "trigger two edges" --expect-fail --stderr --filter "$E" -- -d $K --samples 40 -t CH0=r,CH1=f -O bits
    emu_case "trigger two edges falling first" --expect-fail --stderr --filter "$E" -- -d $K --samples 40 -t CH0=f,CH1=r -O bits

    # Streaming: no compression, the enabled channels in turn.
    # stream_data() (protocol.c:1490-1518) passes on every sample of the
    # transfer and only then checks the limit (1532): 100 samples asked,
    # 256 printed. oxrok stops at the limit; the transactions are the same.
    emu_case "stream" --oracle-differs "protocol.c:1490-1518 passes on all of the transfer in which the limit falls" -- -d $K -c samplerate=1M:continuous=on --samples 100 -O hex
    emu_case "stream three channels" --oracle-differs "protocol.c:1490-1518 passes on all of the transfer in which the limit falls" -- -d $K -C CH0,CH4,CH9 -c samplerate=1M:continuous=on --samples 400 -O hex
    emu_case "stream across transfers" --oracle-differs "protocol.c:1490-1518 passes on all of the transfer in which the limit falls" -- -d $K -c samplerate=200k:continuous=on --samples 1000 -O hex
    emu_case "stream fast" --oracle-differs "protocol.c:1490-1518 passes on all of the transfer in which the limit falls" -- -d $K -c samplerate=20M:continuous=on --samples 300 -O hex
    emu_case "stream trigger ignored" --oracle-differs "protocol.c:1490-1518 passes on all of the transfer in which the limit falls" -- -d $K -c samplerate=1M:continuous=on --samples 64 -t CH0=r -O bits

    # The bare FX2: every run uploads the MCU firmware in the scan and
    # waits for the device to come back (1.8 s), then the bitstream.
    kingst_options mcu=0
    emu_case "bare scan" --keep-header --filter "$C" -- -d $K --scan
    emu_case "bare by ids" --keep-header --filter "$C" -- -d $K:conn=77a1.01a2 --scan
    emu_case "bare capture" -- -d $K -c samplerate=1M --samples 40 -O hex

    # The boot loader refuses the reset hold, a chunk, the release.
    U="$N; /(kingst-la2016|ezusb): /!d; s/^.*(kingst-la2016|ezusb): //"
    for n in 1 2 4; do
        kingst_options mcu=0 stall=loader@$n
        emu_case "fault loader $n" --stderr --filter "$U" -- -d $K --scan
    done
    # The uploaded firmware never runs: the device comes back as the
    # bare FX2, and after 3 s the driver gives up.
    kingst_options mcu=0 fw-dead=1
    emu_case "fault firmware dead" --stderr --filter "$E" -- -d $K --scan
    # The first EEPROM read after the renumeration is refused: the
    # identification there is quiet (show_message FALSE) and tries
    # again 200 ms later, which reads it.
    kingst_options mcu=0 stall=eeprom@1
    emu_case "fault eeprom after the upload" --keep-header --filter "$C" -- -d $K --scan

    # The other models: their rates, memory, base clock, channels.
    kingst_options mcu=1 model=la2016
    emu_case "la2016 scan" --keep-header --filter "$C" -- -d $K --scan
    emu_case "la2016 capture" -- -d $K -c samplerate=2M --samples 50 -O hex
    kingst_options mcu=1 model=la1016
    emu_case "la1016 show" --keep-header --filter "$C" -- -d $K --show
    emu_case "la1016 too fast" --expect-fail --stderr --filter "$E" -- -d $K -c samplerate=200M --samples 8 -O hex
    emu_case "la1016 capture 100M" -- -d $K -c samplerate=100M --samples 50 -O hex
    kingst_options mcu=1 model=la1016a1
    emu_case "la1016a1 trigger" -- -d $K -c samplerate=5M --samples 50 -t CH0=f -O bits
    kingst_options mcu=1 model=la1010a1
    emu_case "la1010a1 show" --keep-header --filter "$C" -- -d $K --show
    emu_case "la1010a1 capture" --oracle-differs "protocol.c:1490-1518 passes on all of the transfer in which the limit falls" -- -d $K -c samplerate=1M --samples 100 -O hex
    emu_case "la1010a1 not continuous" --expect-fail --stderr --filter "$L" -- -d $K -c continuous=off --samples 8 -O hex
    kingst_options mcu=1 model=la5016a1
    emu_case "la5016a1 capture 500M" -- -d $K -c samplerate=500M --samples 50 -O hex
    # 800 MHz / 12207 is 65536, which divider_u16 (protocol.c:757) holds
    # as 0, and protocol.c:758 divides by it: SIGFPE. oxrok refuses the rate.
    emu_case "la5016a1 lowest rate crash" --oracle-bug "protocol.c:757-758 truncates the divider to 0 and divides by it" --expect-fail --stderr --filter "$E" -- -d $K -c samplerate=12207 --samples 8 -O hex
    emu_case "la5016a1 capture 200M" -- -d $K -c samplerate=200M --samples 50 -t CH1=r -O hex
    kingst_options mcu=1 model=la5032a0
    emu_case "la5032a0 scan" --keep-header --filter "$C" -- -d $K --scan
    emu_case "la5032a0 capture" -- -d $K -c samplerate=1M --samples 40 -O hex
    emu_case "la5032a0 trigger" -- -d $K -c samplerate=1M --samples 40 -t CH16=r -O hex
    emu_case "la5032a0 stream" --oracle-differs "protocol.c:1490-1518 passes on all of the transfer in which the limit falls" -- -d $K -C CH0,CH17 -c samplerate=1M:continuous=on --samples 64 -O hex

    # The bitstream stays loaded: the check reads the FPGA and reuses
    # it. With a PWM output left running, or a run state the check does
    # not expect, the driver uploads it again.
    kingst_options mcu=1 fpga=1
    emu_case "loaded scan" --keep-header --filter "$C" -- -d $K --scan
    # Recorded: with the bitstream loaded the replay needs no firmware file.
    emu_case "loaded capture" --record -- -d $K -c samplerate=1M --samples 30 -O hex
    emu_case "loaded trigger" --record -- -d $K -c samplerate=1M --samples 40 -t CH0=r -O bits
    # 256 samples are a whole transfer: here the limit cuts nothing.
    emu_case "loaded stream" --record -- -d $K -c samplerate=1M:continuous=on --samples 256 -O hex
    kingst_options mcu=1 fpga=1 pwm=1
    emu_case "loaded pwm running" --keep-header --filter "$C" -- -d $K --scan
    kingst_options mcu=1 fpga=1 run-state=85e2
    emu_case "loaded busy" --keep-header --filter "$C" -- -d $K --scan
    kingst_options mcu=1 fpga=1 run-state=1231
    emu_case "loaded odd run state" --stderr --filter "$E" -- -d $K --scan

    # The EEPROM: the magic's other copies and their checks.
    kingst_options mcu=1 eeprom-id=02fd01fe00000000
    emu_case "eeprom primary magic" --stderr --filter "$N; /magic/!d; $E" -- -l 4 -d $K --scan
    kingst_options mcu=1 eeprom-id=08f7000000000000
    emu_case "eeprom primary byte" --stderr --filter "$N; /magic/!d; $E" -- -l 4 -d $K --scan
    kingst_options mcu=1 eeprom-id=0000000008f70000
    emu_case "eeprom backup byte" --stderr --filter "$N; /magic/!d; $E" -- -l 4 -d $K --scan
    kingst_options mcu=1 eeprom-id=ffffffffffffffff
    emu_case "eeprom no magic" --stderr --filter "$E" -- -d $K --scan
    kingst_options mcu=1 eeprom-id=30cf000030cf0000
    emu_case "eeprom unknown magic" --stderr --filter "$E" -- -d $K --scan
    # The LA2016 A2 (magic 0x0b, 0x10): its bitstream is no stand-in.
    kingst_options mcu=1 eeprom-id=0bf410ef0bf410ef
    emu_case "eeprom model without bitstream" --stderr --filter "$E" -- -d $K --scan
    kingst_options mcu=1 date=20040000
    emu_case "eeprom bad date" --stderr --filter "$E" -- -d $K --scan

    # Faults in the scan: the identification and the bitstream.
    kingst_options mcu=1 stall=eeprom@1
    emu_case "fault date read" --stderr --filter "$E" -- -d $K --scan
    kingst_options mcu=1 stall=eeprom@2
    emu_case "fault magic read" --stderr --filter "$E" -- -d $K --scan
    kingst_options mcu=1 stall=fpga-init-in@1
    emu_case "fault fpga check" --keep-header --filter "$C" -- -d $K --scan
    kingst_options mcu=1 stall=fpga-init-out
    emu_case "fault fpga init" --stderr --filter "$E" -- -d $K --scan
    kingst_options mcu=1 stall=bitstream@3
    emu_case "fault bitstream stall" --stderr --filter "$E" -- -d $K --scan
    kingst_options mcu=1 short-bitstream=1
    emu_case "fault bitstream short" --stderr --filter "$E" -- -d $K --scan
    kingst_options mcu=1 stall=fpga-init-in@2
    emu_case "fault fpga response" --stderr --filter "$E" -- -d $K --scan
    kingst_options mcu=1 fpga-init=5
    emu_case "fault fpga response value" --stderr --filter "$E" -- -d $K --scan
    kingst_options mcu=1 stall=fpga-enable
    emu_case "fault fpga enable" --stderr --filter "$E" -- -d $K --scan
    # The register dump of the spew level (-l 5) is refused.
    kingst_options mcu=1 stall=read:0x00@1
    emu_case "fault verbose dump" --stderr --filter "$N; /registers space/!d; s/^.*kingst-la2016: //" -- -l 5 -d $K --scan
    kingst_options mcu=1 stall=read:0x00@1
    # run_state() returns the failure as a state (protocol.c:910-913,
    # SR_ERR_IO as 0xfff5), and la2016_init_hardware() warns about it
    # (protocol.c:1892-1895). oxrok logs the failure alone. sigrok-cli's
    # stdout is what the .oracle keeps: both find the device.
    emu_case "fault init run state" --oracle-differs "protocol.c:912 returns the failed read as a run state" --stderr --filter "$N; /kingst-la2016: |^kingst-la2016:conn/!d; s/^.*kingst-la2016: //; $C" -- -d $K --scan
    kingst_options mcu=1 stall=bulk-reset@1
    emu_case "fault init bulk reset" --stderr --filter "$E" -- -d $K --scan
    kingst_options mcu=1 fpga=1 stall=read:0x00@1
    emu_case "fault check run state" --keep-header --filter "$C" -- -d $K --scan
    kingst_options mcu=1 fpga=1 stall=read:0x02@1
    emu_case "fault check pwm" --keep-header --filter "$C" -- -d $K --scan

    # Faults after the scan: the open, the setup, the start, the
    # download.
    kingst_options mcu=1 stall=write:0x02@1
    emu_case "fault open pwm" --expect-fail --stderr --filter "$E" -- -d $K --samples 8 -O hex
    kingst_options mcu=1 stall=write:0x02@3
    emu_case "fault pwm set" --expect-fail --stderr --filter "$E" -- -d $K -g PWM1 --set -c enabled=on
    kingst_options mcu=1 stall=write:0x02@3
    emu_case "fault pwm frequency" --expect-fail --stderr --filter "$E" -- -d $K -g PWM2 --set -c output_frequency=5000
    kingst_options mcu=1 stall=write:0x02@3
    emu_case "fault pwm duty" --expect-fail --stderr --filter "$E" -- -d $K -g PWM2 --set -c output_duty_cycle=30
    kingst_options mcu=1 stall=write:0x70
    emu_case "fault pwm parameters" --expect-fail --stderr --filter "$E" -- -d $K -g PWM1 --set -c enabled=on
    kingst_options mcu=1 stall=write:0x02@4
    emu_case "fault pwm enable" --expect-fail --stderr --filter "$E" -- -d $K -g PWM1 --set -c enabled=on
    for f in write:0x68 write:0x03 write:0x20 write:0x10 write:0x00@1; do
        kingst_options mcu=1 stall=$f
        emu_case "fault setup $f" --expect-fail --stderr --filter "$E" -- -d $K --samples 8 -O hex
    done
    kingst_options mcu=1 stall=read:0x10
    emu_case "fault capture info" --stderr --filter "$E" -- -d $K --samples 8 -O hex
    kingst_options mcu=1 stall=bulk-reset@2
    emu_case "fault download bulk reset" --stderr --filter "$E" -- -d $K --samples 8 -O hex
    kingst_options mcu=1 stall=write:0x08
    emu_case "fault download window" --stderr --filter "$E" -- -d $K --samples 8 -O hex
    # A libsigrok bug: when la2016_receive_data() fails after the bulk
    # transfers were submitted (a refused bulk start of the download,
    # or of a stream, or its run mode), it returns FALSE with the eight
    # transfers still in flight and without ending the session feed;
    # dev_close() then frees them (la2016_release_resources()) and
    # libusb aborts on a freed mutex
    # (usbi_mutex_lock: Assertion `pthread_mutex_lock(mutex) == 0').
    # sigrok-cli dies with SIGABRT. oxrok logs the driver's lines, cancels
    # the transfers and ends the capture (exit 0, as sigrok-cli after a
    # failed download start without transfers: "fault capture info").
    kingst_options mcu=1 stall=bulk-start@1
    emu_case "fault download start" --oracle-bug "protocol.c:1654-1657 leaves the bulk transfers in flight; dev_close() frees them (api.c:700, protocol.c:1049) and libusb aborts" --stderr --filter "$E" -- -d $K --samples 8 -O hex
    kingst_options mcu=1 stall=bulk-reset@2
    emu_case "fault stream bulk reset" --expect-fail --stderr --filter "$E" -- -d $K -c continuous=on --samples 8 -O hex
    kingst_options mcu=1 stall=write:0x00@1
    emu_case "fault stream run" --oracle-bug "protocol.c:1613-1617 leaves the bulk transfers in flight; dev_close() frees them (api.c:700, protocol.c:1049) and libusb aborts" --stderr --filter "$E" -- -d $K -c continuous=on --samples 8 -O hex
    kingst_options mcu=1 stall=bulk-start@1
    emu_case "fault stream start" --oracle-bug "protocol.c:1619-1623 leaves the bulk transfers in flight; dev_close() frees them (api.c:700, protocol.c:1049) and libusb aborts" --stderr --filter "$E" -- -d $K -c continuous=on --samples 8 -O hex
    kingst_options mcu=1 extra-packets=2
    emu_case "fault packet count" --stderr --filter "$E" -- -d $K --samples 8 -O hex
    kingst_options mcu=1 empty=2 stall-in=1
    emu_case "fault empty and stalled transfers" -- -d $K --samples 30 -O hex
    kingst_options mcu=1 empty=2 stall-in=1
    emu_case "fault stream empty and stalled" --oracle-differs "protocol.c:1490-1518 passes on all of the transfer in which the limit falls" -- -d $K -c continuous=on --samples 30 -O hex
    # A capture of several polls (1000 samples each at 20 kHz): the run
    # state is pre-trigger twice, waiting, then post-trigger twice.
    kingst_options mcu=1
    emu_case "slow capture" -- -d $K -C CH0,CH1 -c samplerate=20k --samples 5000 -t CH1=f -O hex
    # The device sends more than the window, or holds more samples than
    # asked: the driver stops at the window, or at the sample limit
    # after the transfer that reaches it, whose samples it passes on
    # in full (sigrok-cli prints them all).
    kingst_options mcu=1 extra-bytes=32
    emu_case "fault download overrun" -- -d $K --samples 30 -O hex
    kingst_options mcu=1 more-samples=2000000
    emu_case "fault more samples" --oracle-differs "send_chunk() (protocol.c:1381-1413) passes on all of the transfer in which the limit falls" --filter "1,/^CH15/!d" -- -d $K --samples 30 -O hex
    # A failed poll of the run state: run_state() returns SR_ERR_IO as
    # a u16 (0xfff5), whose idle bit is set, so the driver downloads a
    # capture that has not finished: the capture info is all zeros,
    # nothing comes, the transfers time out and the frame is empty.
    kingst_options mcu=1 stall=read:0x00@2
    # oxrok ends the capture there, halting the device.
    emu_case "fault poll run state" --oracle-differs "protocol.c:912 returns the failed read as a run state, whose idle bit starts a download" --no-output -- -d $K --samples 8 -O hex
    # Unplugged once the first of two download transfers has come.
    kingst_options mcu=1
    emu_case "unplug during download" --filter '1,/^CH15/!d' --unplug in=6:1 -- -d $K -c samplerate=100M --samples 2000000 -O hex
    kingst_options
}
