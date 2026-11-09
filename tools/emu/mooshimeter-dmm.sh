# The Mooshimeter (emu/src/models/mooshimeter-dmm.rs), a Bluetooth LE peripheral
# behind a virtual HCI controller on /dev/vhci. Sourced by tools/diffcli.sh --emu;
# see docs/USB.md, "A Bluetooth LE model". The tools reach it through the kernel by
# its address, D0:5F:B8:4D:00:01. sigrok-cli splits -d and --config at colons, so
# the cases write the address with dashes and channel_config's fields with
# semicolons, both of which libsigrok takes too.
EMU_MODEL=mooshimeter-dmm
EMU_BLE=1
EMU_DRIVER=mooshimeter-dmm
EMU_GCOV=mooshimeter-dmm
EMU_GCOV_SHARED="src/bt/bt_bluez.c"

emu_cases() {
    D=mooshimeter-dmm
    A=D0-5F-B8-4D-00-01
    T=$D:conn=$A

    # The scan connects and disconnects, and takes whatever answers for a meter.
    emu_options
    emu_case "scan" --keep-header -- -d $T --scan
    emu_case "scan, the address dense" --keep-header -- -d $D:conn=D05FB84D0001 --scan
    emu_case "scan, the address in lower case" --keep-header -- -d $D:conn=d0-5f-b8-4d-00-01 --scan
    emu_case "scan, the address with colons" --stderr --expect-fail -- -d $D:conn=D0:5F:B8:4D:00:01 --scan
    emu_case "scan, the address too short" --keep-header --stderr -- -d $D:conn=D0-5F-B8-4D-00 --scan
    emu_case "scan, the address too long" --keep-header --stderr -- -d $D:conn=D0-5F-B8-4D-00-01-02 --scan
    emu_case "scan with no conn" --keep-header --no-output -- -d $D --scan
    # Nobody at the address, and a meter that never completes a connection:
    # the kernel tries for 40 s (L2CAP_CONN_TIMEOUT).
    emu_case "scan, another address" --keep-header --no-output --timeout 100 -- -d $D:conn=D0-5F-B8-4D-00-02 --scan
    emu_options refuse=1
    emu_case "scan of a meter that refuses" --keep-header --no-output --timeout 100 -- -d $T --scan
    emu_options

    # What a show and a get read: the tree, through an open.
    emu_case "show" --keep-header -- -d $T --show
    emu_case "get samplerate" -- -d $T --get samplerate
    emu_case "get avg_samples" -- -d $T --get avg_samples
    emu_case "get limit_samples" --stderr --expect-fail -- -d $T --get limit_samples
    emu_case "get channel_config" --stderr --expect-fail -- -d $T --get channel_config
    emu_case "set samplerate" --no-output -- -d $T --config samplerate=1000 --set
    emu_case "set samplerate between two" --no-output -- -d $T --config samplerate=300 --set
    emu_case "set samplerate above all" --no-output -- -d $T --config samplerate=20000 --set
    emu_case "set avg_samples" --no-output -- -d $T --config avg_samples=256 --set

    # Captures, at the meter's own pace: a reading every 512 ms.
    emu_case "two samples" -- -d $T --samples 2
    emu_case "one sample ends inside a reading" -- -d $T --samples 1
    emu_case "five samples" -- -d $T --samples 5
    emu_case "CH2 alone" -- -d $T -C CH2 --samples 2
    emu_case "power too" -- -d $T -C CH1,CH2,P --samples 6
    emu_case "power alone" -- -d $T -C P --samples 2
    emu_case "a time limit" -- -d $T --time 800
    emu_case "a faster rate" -- -d $T --config samplerate=4000:avg_samples=32 --samples 6
    emu_case "two configs, one capture" -- -d $T --config samplerate=250 --config avg_samples=128 --samples 2
    emu_case "analog output" -- -d $T --samples 2 -O analog
    emu_case "csv output" -- -d $T -C CH1,CH2,P --samples 6 -O csv

    # channel_config: CH1[,CH2], each MODE[;RANGE[;ANALYSIS]].
    for cfg in "A" "Current;10" "A;10;RMS" "A;0;AC" "a;2;dc" "T" "K;350" "Temperature" \
        "Resistance" "Ohm;10000" "W;20000" "R;5e6" "Diode" "D;1" "Aux" "LV;0.3" "LV;0.2;RMS" \
        "A,V" "A,V;60" "A,V;600" "A,V;100" "A,V;60;RMS" "A,T" "A,Temperature;350" \
        "A,Resistance" "A,Ohm;1000" "A,Diode" "A,Aux;1.2" "A,LV;0.1;AC" \
        "A;10;RMS/V;600;AC" "Xyz" "A,Xyz" ",V;600" "A;x" "A;-3"; do
        emu_case "channel_config=$cfg" -- -d $T --config "channel_config=$cfg" --samples 2
    done
    emu_case "channel_config, CH2 left out" -- -d $T --config "channel_config=A," --samples 2
    emu_case "channel_config, a range alone" -- -d $T --config "channel_config=;5" --samples 2
    emu_case "channel_config with a colon" --stderr --expect-fail -- -d $T --config channel_config=A:10 --samples 2
    # The shared input has one owner.
    emu_case "channel_config=R,R" --stderr --expect-fail -- -d $T --config channel_config=R,R --samples 2
    emu_case "channel_config=D,LV" --stderr --expect-fail -- -d $T --config channel_config=D,LV --samples 2
    emu_case "channel_config=LV,D" --stderr --expect-fail -- -d $T --config channel_config=LV,D --samples 2

    # Buffers: DEPTH samples of each channel per reading.
    emu_case "a buffer" -- -d $T --config "channel_config=A;10;Buffer,V;60;Samples:avg_samples=32" --samples 64
    emu_case "a buffer of CH2" -- -d $T -C CH2 --config "channel_config=A,V;60;Buffer:avg_samples=32" --samples 32
    emu_case "an autoranging buffer" -- -d $T -C CH2 --config "channel_config=A,V;0;Buffer:avg_samples=32" --samples 64

    # Autorange: a reading out of its range moves the range, unasked.
    emu_options ch2=12.5,75,12.5,-350
    emu_case "autorange up and down" -- -d $T -C CH2 --samples 4
    emu_options ch2=75
    emu_case "a fixed range stays" -- -d $T -C CH2 --config "channel_config=A,V;60" --samples 2

    # The serial layer: numbers that wrap, come out of order, twice, or not at all.
    emu_options seq=250
    emu_case "sequence numbers wrap" -- -d $T --samples 2
    emu_options swap=3
    emu_case "a notification out of order" -- -d $T --samples 2
    emu_options dup=3
    emu_case "a notification twice" -- -d $T --samples 2
    emu_options lose=37
    emu_case "a lost notification" --hangs --no-output --timeout 8 -- -d $T --samples 4
    emu_options lose=3
    emu_case "a lost notification in the tree" --no-output --expect-fail --timeout 90 -- -d $T --samples 2

    # What the meter says unasked.
    emu_options battery=1
    emu_case "the battery, unasked" -- -d $T --samples 4
    emu_options diag=BAD_DATA
    emu_case "a diagnostic" --stderr -- -d $T --samples 2

    # A meter that fails the handshake.
    emu_options crc=bad
    emu_case "a CRC echoed wrong" --no-output --expect-fail -- -d $T --samples 2
    emu_options tree=garbage
    emu_case "a tree that is not zlib" --no-output --expect-fail -- -d $T --samples 2
    emu_options tree=no-pwr
    emu_case "a tree with no power" --stderr -- -d $T --samples 2

    # A meter that stops answering, or goes.
    emu_options mute-at=13
    emu_case "no answer to the start" --no-output --expect-fail -- -d $T --samples 2
    emu_options mute-at=0
    emu_case "no answer to the tree" --no-output --expect-fail --timeout 90 -- -d $T --samples 2
    emu_options stop-at=0
    emu_case "no reading comes" --hangs --no-output --timeout 8 -- -d $T --time 2500
    emu_options
    emu_case "the meter goes" --unplug ble-notify=37 --hangs --no-output --timeout 8 -- -d $T --samples 4
    emu_options idle=2 period=3000
    emu_case "a meter that drops a quiet host" --hangs --no-output --timeout 10 -- -d $T --time 4000
    emu_options
    # The heartbeat comes every 10 s.
    emu_case "eleven seconds" --timeout 90 -- -d $T -C CH1 --time 11000
}
