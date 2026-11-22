# The WCH CH32H417 logic analyzers, the WeAct Studio Logic Analyzer V2
# among them (emu/src/models/wch-ch32h417-libusb.rs): a vendor USB device,
# 1a86:5537, with no firmware to upload. Sourced by tools/diffcli.sh --emu;
# see docs/USB.md, "Writing a device model".
#
# The C driver is not sigrok's: it is the vendor's, which the oracle
# carries (oxrok/libsigrok, src/hardware/wch-ch32h417-libusb), so --oracle
# needs an oracle built from that branch.
#
# NEVER RUN. These cases were written from a reading of the C driver, on a
# machine where the harness could not attach a device. The first run with
# the oracle decides each one: a case marked --oracle-differs or
# --oxrok-fails here is what the reading expects, not what a run showed,
# and the plain ones may turn out to differ as well. One difference is
# known to reach every capture: the frame marks. sigrok-cli prints the C
# driver's (no FRAME-BEGIN before an untriggered frame, one more after the
# last FRAME-END), oxrok whole frames, and --digest takes the output before
# the filter. That is to be settled before the first bless (docs/ROADMAP.md):
# the C driver's frames fixed in the oracle's fork, or oxrok sending the
# C driver's. Until tests/emu holds
# goldens the model is skipped by name, which a run says; a bless runs it.
EMU_MODEL=wch-ch32h417-libusb
EMU_DRIVER=wch-ch32h417-libusb
# oxrok has the driver: its cases must run full.
EMU_FULL=wch-ch32h417-libusb
EMU_GCOV_SHARED="src/usb.c"
[ -d "${EMU_GOLDEN:-}/wch-ch32h417-libusb" ] || [ "${EMU_BLESS:-0}" != 0 ] \
    || EMU_SKIP="no goldens yet: EMU_BLESS=1 sh tools/diffcli.sh --oracle --emu-model wch-ch32h417-libusb writes them, and each wants reading"

emu_cases() {
    W=wch-ch32h417-libusb
    # oxrok's log prefix as sigrok-cli's: only the prefix goes, the text
    # stays compared.
    N='s/^oxrok: [a-z]+: oxrok::drivers::wch_ch32h417_libusb[a-z0-9_:]*: /sr: wch-ch32h417-libusb: /'
    # The driver's own messages on stderr, without sigrok-cli's prefix.
    E="$N; /wch-ch32h417-libusb: /!d; s/.*wch-ch32h417-libusb: //"
    # The last stderr line, sigrok-cli's own.
    L='$!d'
    # oxrok's wording of a refused option as sigrok-cli's.
    A='s/(invalid argument).*$/\1./'
    # The place on the bus, in --scan and --show.
    C='s#usb/[0-9]+-[0-9.]+#usb/BUS-PORT#'
    # What a capture is held to: the first 64 samples, the trigger marker
    # and, through --digest, the SHA-256 of the whole output.
    CH=D0,D1,D2,D7,D8,D15
    B='1,8b; /^T:/b; d'
    S=1000

    # A V2 on a USB 2 port, the model's default.
    emu_options
    emu_case "scan" --keep-header --filter "$C" -- -d $W --scan
    emu_case "show" --keep-header --filter "$C" -- -d $W --show
    # config_list() gives the ADC's options for any group, on a board
    # without an ADC too.
    emu_case "show Logic" --keep-header --filter "$C" -- -d $W -g Logic --show
    emu_case "default capture" --record --digest --filter "$B" -- -d $W -C $CH --samples $S -O bits
    emu_case "all channels" --digest --filter '1,40!d' -- -d $W --samples $S -O hex
    # The rate goes down as its place in the table shown.
    for r in 20M 16M 12M 8M 6M 4M; do
        emu_case "samplerate $r" --digest --filter '1,3!d' -- -d $W -C D0 -c samplerate=$r --samples $S -O bits
    done
    emu_case "a rate of the other table" --expect-fail --stderr --filter "$L; $A" -- -d $W -c samplerate=40M --samples $S
    emu_case "a rate of no table" --expect-fail --stderr --filter "$L; $A" -- -d $W -c samplerate=1M --samples $S
    # Eight channels are a byte a sample, with their own rates; the count
    # is set before the rate, as the table follows it.
    emu_case "eight channels" --digest --filter "$B" -- -d $W -c logic_channels=8 -C D0,D1,D7 --samples $S -O bits
    emu_case "eight channels 40M" --digest --filter '1,3!d' -- -d $W -c logic_channels=8:samplerate=40M -C D0 --samples $S -O bits
    emu_case "nine channels" --digest --filter '1,3!d' -- -d $W -c logic_channels=9 -C D8 --samples $S -O bits
    for n in 0 17; do
        emu_case "logic_channels $n" --expect-fail --stderr --filter "$L; $A" -- -d $W -c logic_channels=$n --samples $S
    done
    # More samples than one transfer holds, and all the model sends.
    emu_case "200k samples" --digest --filter '1~4096!d' -- -d $W -C D0,D15 --samples 200k -O bits
    emu_case "time limit" --digest --filter '1,3!d' -- -d $W -C D0 -c samplerate=4M --time 10 -O bits

    # The threshold: the level on the pins, sent as the level at the input.
    for t in 0.6 1.0 1.5; do
        emu_case "threshold $t" --digest --filter '1,3!d' -- -d $W -c voltage_threshold=$t-$t -C D0 --samples $S -O bits
    done
    for t in 0.5 1.6; do
        emu_case "threshold $t" --expect-fail --stderr --filter "$L; $A" -- -d $W -c voltage_threshold=$t-$t --samples $S
    done
    emu_case "led brightness" --digest --filter '1,3!d' -- -d $W -c led_brightness=80 -C D0 --samples $S -O bits
    emu_case "led brightness 101" --expect-fail --stderr --filter "$L; $A" -- -d $W -c led_brightness=101 --samples $S
    emu_case "key start stop" --digest --filter '1,3!d' -- -d $W -c key_start_stop=on -C D0 --samples $S -O bits
    emu_case "no ADC mode" --expect-fail --stderr --filter "$L; $A" -- -d $W -c "pattern=ADC" --samples $S
    emu_case "captureratio above 100" --expect-fail --stderr --filter "$L; $A" -- -d $W -c captureratio=150 --samples $S

    # What a get asks the device, and what it does not.
    for k in samplerate limit_samples limit_frames captureratio pattern voltage_threshold \
        hardware_version usb_version link_stop_reason key_start_stop key_pressed led_brightness; do
        emu_case "get $k" -- -d $W --get $k
    done
    emu_case "get conn" --filter "$C" -- -d $W --get conn
    emu_case "get threshold set" -- -d $W -c voltage_threshold=1.2-1.2 --get voltage_threshold
    emu_case "get adc_precision" -- -d $W -g Logic --get adc_precision

    # The soft trigger: the vendor's, which waits for the pre-trigger
    # samples before it matches, so the ratio moves the trigger.
    emu_case "trigger D1 rising" --record --digest --filter "$B" -- -d $W -C $CH -t D1=r --samples $S -O bits
    emu_case "trigger D2 falling" --digest --filter "$B" -- -d $W -C $CH -t D2=f --samples $S -O bits
    emu_case "trigger D7 edge" --digest --filter "$B" -- -d $W -C $CH -t D7=e --samples $S -O bits
    emu_case "trigger levels" --digest --filter "$B" -- -d $W -C $CH -t D0=1,D1=0,D2=1 --samples $S -O bits
    emu_case "trigger two stages" --digest --filter "$B" -- -d $W -C $CH -t D2=01 --samples $S -O bits
    for ratio in 10 50 100; do
        emu_case "trigger ratio $ratio" --digest --filter "$B" -- -d $W -C $CH -t D8=r -c captureratio=$ratio --samples $S -O bits
    done
    emu_case "trigger eight channels" --digest --filter "$B" -- -d $W -c logic_channels=8 -C D0,D7 -t D7=r -c captureratio=20 --samples $S -O bits
    emu_case "trigger on a disabled channel" --digest --filter "$B" -- -d $W -C D0 -t D8=r --samples $S -O bits
    emu_case "trigger position" --digest --filter '/^T/!d' -- -d $W -C D0,D8 -t D8=r -c captureratio=30 --samples $S -O csv:trigger=true:header=false

    # Frames. The C driver begins no frame without a trigger, begins one
    # after the pre-trigger samples, and begins an empty one after the
    # last frame's end; oxrok's frames are whole. The outputs here show no
    # frame marks, so the samples are what is compared.
    emu_case "two frames" --digest --filter "$B" -- -d $W -C $CH --samples $S --frames 2 -O bits
    emu_case "three frames triggered" --digest --filter "$B" -- -d $W -C $CH -t D7=r -c captureratio=10 --samples $S --frames 3 -O bits

    # Selection. The C driver logs that it ignores conn and lists every
    # board; oxrok takes it.
    emu_case "by address" --digest --filter "$B" -- -d $W:conn=$EMU_USB -C $CH --samples $S -O bits
    emu_case "by ids" --digest --filter "$B" -- -d $W:conn=1a86.5537 -C $CH --samples $S -O bits
    emu_case "another address" --oracle-differs "api.c:282-283: the scan ignores conn and lists every board" --keep-header --no-output -- -d $W:conn=1.127 --scan

    # A USB 3 link, by bcdUSB: the long table, the same for either width.
    emu_options usb=3
    emu_case "usb3 show" --keep-header --filter "$C" -- -d $W --show
    emu_case "usb3 default capture" --digest --filter "$B" -- -d $W -C $CH --samples $S -O bits
    for r in 200M 156.25M 78.125M 4M; do
        emu_case "usb3 samplerate $r" --digest --filter '1,3!d' -- -d $W -C D0 -c samplerate=$r --samples $S -O bits
    done
    emu_case "usb3 eight channels" --digest --filter '1,3!d' -- -d $W -c logic_channels=8:samplerate=200M -C D0 --samples $S -O bits
    emu_case "usb3 get usb_version" -- -d $W --get usb_version

    # Firmware 0.1: no key count, so no key options.
    emu_options key=0 fw=1
    emu_case "fw 0.1 show" --keep-header --filter "$C" -- -d $W --show
    emu_case "fw 0.1 capture" --digest --filter "$B" -- -d $W -C $CH --samples $S -O bits
    emu_case "fw 0.1 key start stop" --expect-fail --stderr --filter "$L" -- -d $W -c key_start_stop=on --samples $S
    emu_case "fw 0.1 get hardware_version" -- -d $W --get hardware_version

    # The original board: an ADC, two analog channels, one rate over USB 2,
    # the threshold as DAC counts.
    emu_options serial=0123456789ab
    emu_case "original scan" --keep-header --filter "$C" -- -d $W --scan
    emu_case "original show" --keep-header --filter "$C" -- -d $W --show
    emu_case "original show A0" --keep-header --filter "$C" -- -d $W -g A0 --show
    emu_case "original capture" --digest --filter "$B" -- -d $W -C $CH --samples $S -O bits
    emu_case "original threshold 1.5" --digest --filter '1,3!d' -- -d $W -c voltage_threshold=1.5-1.5 -C D0 --samples $S -O bits
    emu_case "original 16M" --expect-fail --stderr --filter "$L; $A" -- -d $W -c samplerate=16M --samples $S
    emu_options serial=0123456789ab usb=3
    emu_case "original usb3 show" --keep-header --filter "$C" -- -d $W --show
    # The ADC mode. With both analog channels enabled the C driver says its
    # one sample a point belongs to two channels, and the output reads past
    # the buffer; oxrok sends the selected input's samples as that input's.
    emu_case "adc capture A0" --oracle-differs "protocol.c:918-923: one float a sample for every enabled analog channel's" --digest --filter '1,8!d' -- -d $W -c "pattern=ADC" -C A0,A1 --samples $S -O analog
    emu_case "adc capture one channel" --digest --filter '1,8!d' -- -d $W -c "pattern=ADC" -C A0 --samples $S -O analog
    emu_case "adc 10 bit input 1" --digest --filter '1,8!d' -- -d $W -c "pattern=ADC:adc_precision=10:adc_channel=1:samplerate=10M" -C A1 --samples $S -O analog
    emu_case "adc precision 9" --expect-fail --stderr --filter "$L; $A" -- -d $W -c adc_precision=9 --samples $S
    emu_case "adc trigger" --oracle-differs "protocol.c:1075: the trigger runs on the raw ADC bytes and its pre-trigger samples go out as logic" --expect-fail --stderr --filter "$L" -- -d $W -c "pattern=ADC" -C A0 -t D0=r --samples $S -O analog

    # No board the driver knows.
    emu_options serial=7f00000000
    emu_case "unknown serial" --keep-header --no-output -- -d $W --scan

    # Faults. A refused or unanswered parameter set fails the start.
    emu_options status=a0:01
    emu_case "parameters refused" --expect-fail --stderr --filter "$E" -- -d $W --samples $S
    emu_options silent=a0
    emu_case "parameters unanswered" --expect-fail --stderr --filter "$E" --timeout 10 -- -d $W --samples $S
    emu_options short=a0
    emu_case "parameters answered short" --digest --filter '1,3!d' -- -d $W -C D0 --samples $S -O bits
    emu_options garbage=a0
    emu_case "parameters answered as another command" --expect-fail --stderr --filter "$E" -- -d $W --samples $S
    emu_options stall=a0
    emu_case "parameters stalled" --expect-fail --stderr --filter "$E" -- -d $W --samples $S
    emu_options silent=a4
    emu_case "no key count at the open" --keep-header --filter "$C" --timeout 10 -- -d $W --show
    emu_options silent=ac
    emu_case "version unanswered" --timeout 10 -- -d $W --get hardware_version
    emu_options status=a1:02
    emu_case "threshold refused" --expect-fail --stderr --filter "$E" -- -d $W -c voltage_threshold=1.0-1.0 --samples $S
    # Answers nobody read, which the drain at the scan meets.
    emu_options unread=3
    emu_case "unread answers" --digest --filter "$B" -- -d $W -C $CH --samples $S -O bits
    emu_options unread=20
    emu_case "more unread answers than the drain takes" --stderr --filter "$E" -- -d $W --samples $S -O bits

    # A stream that ends before the limit: the driver's path for a link
    # that cannot carry the rate. The C driver ends the capture as any
    # other and exits 0; oxrok fails it.
    emu_options transfers=1
    emu_case "the samples stop" --oxrok-fails "the device sent no samples for 500 ms" --digest --filter '1,3!d' --timeout 10 -- -d $W -C D0 --samples 500k -O bits
    emu_options transfers=0
    emu_case "no samples at all" --oxrok-fails "the device sent no samples for 500 ms" --no-output --timeout 10 -- -d $W -C D0 --samples $S -O bits
    # The board's key ends a capture that allowed it, which is no failure.
    emu_options transfers=4 press=1
    emu_case "the key ends the capture" --digest --filter '1,3!d' --timeout 10 -- -d $W -c key_start_stop=on -C D0 --samples 500k -O bits
    emu_case "a key press that may not end it" --digest --filter '1,3!d' --timeout 10 -- -d $W -C D0 --samples 500k -O bits
    emu_case "key pressed" -- -d $W --get key_pressed
    # A sample transfer that fails. The C ring puts it back and goes on;
    # oxrok ends the capture with the error.
    emu_options transfers=3 stall-in=2
    emu_case "a sample transfer stalls" --oxrok-fails "stalled" --digest --filter '1,3!d' --timeout 10 -- -d $W -C D0 --samples 200k -O bits

    emu_options
}
