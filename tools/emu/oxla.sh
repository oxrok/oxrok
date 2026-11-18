# The oxla logic analyzer gateware on a Tang Nano 20K
# (emu/src/models/oxla.rs) on the CDC-ACM shell, behind a UART at
# 3000000/8n1. Sourced by tools/diffcli.sh --emu; see docs/USB.md, "Writing
# a serial model".
#
# libsigrok has no such driver, so there is no oracle: every case is
# --no-oracle-only, oxrok alone against goldens blessed from oxrok
# (EMU_BLESS=missing sh tools/diffcli.sh --emu-model oxla). What holds the
# driver to the device is the board (docs/DEVICES.md); what these hold is
# the driver to itself, and to the model's pedantic checks.
EMU_MODEL=oxla
EMU_CDC=1
EMU_DRIVER=oxla
EMU_FULL=$EMU_DRIVER
# The goldens are not blessed yet (docs/ROADMAP.md): with none to hold
# oxrok to, every case would fail for the lack of one. Until tests/emu/oxla
# is there the model is skipped by name, which a run says; a bless runs it.
[ -d "${EMU_GOLDEN:-}/oxla" ] || [ "${EMU_BLESS:-0}" != 0 ] \
    || EMU_SKIP="no goldens yet: EMU_BLESS=missing sh tools/diffcli.sh --emu-model oxla writes them, and each wants reading"

# oxla_case NAME [emu_case's flags] -- ARGS...
oxla_case() {
    oc_name=$1
    shift
    emu_case "$oc_name" --no-oracle-only "$@"
}

emu_cases() {
    emu_conns
    # The counter pattern, which says what a capture must hold.
    P="-c pattern=Internal"
    CSV="-O csv:trigger=true:header=false"

    # What a scan finds: the bitstream, by its ID, and its clocks.
    emu_options
    oxla_case "scan" --keep-header -- -d $O --scan
    oxla_case "show" --keep-header -- -d $O --show
    emu_options channels=14 spc=2
    oxla_case "scan la14-ddr" --keep-header -- -d $O --scan
    oxla_case "show la14-ddr" --keep-header -- -d $O --show
    emu_options channels=8 spc=2
    oxla_case "show la8-ddr" --keep-header -- -d $O --show
    emu_options channels=8
    oxla_case "show la8-rle" --keep-header -- -d $O --show
    emu_options channels=8 rle=0 adc=1
    oxla_case "show la8-adc" --keep-header -- -d $O --show
    oxla_case "get samplerate la8-adc" -- -d $O --get samplerate
    emu_options rle=0
    oxla_case "show la16" --keep-header -- -d $O --show
    # A bitstream from before the host could change the clock.
    emu_options clock=108000000
    oxla_case "show one clock" --keep-header -- -d $O --show
    emu_options clock=50000000
    oxla_case "show another clock" --keep-header -- -d $O --show
    oxla_case "another clock capture" -- -d $O $P -c samplerate=1m --samples 8 -O hex
    # What is no oxla device.
    emu_options id=none
    oxla_case "scan no ID" --keep-header --no-output --timeout 20 -- -d $O --scan
    emu_options id=31534c4f
    oxla_case "scan a short answer" --keep-header --no-output --timeout 20 -- -d $O --scan
    emu_options id=31534c4f0101100100f36f0600002000
    oxla_case "scan another ID" --keep-header --stderr -- -d $O --scan
    emu_options id=4f584c410201100100f36f0600002000
    oxla_case "scan protocol 2" --keep-header --stderr -- -d $O --scan
    emu_options id=4f584c410101120100f36f0600002000
    oxla_case "scan 18 channels" --keep-header --stderr -- -d $O --scan
    # The SDRAM does not come back from a change of clock.
    emu_options settle=100000
    oxla_case "scan clock never ready" --keep-header --stderr --expect-fail --timeout 30 -- -d $O --scan
    emu_options settle=3
    oxla_case "scan clock ready late" --keep-header -- -d $O --scan

    # conn and serialcomm.
    emu_options
    oxla_case "no such node" --keep-header --stderr --expect-fail -- -d oxla:conn=/dev/ttyACM97 --scan
    oxla_case "no conn" --keep-header --no-output -- -d oxla --scan
    oxla_case "serialcomm not the UART's" --keep-header --no-output --timeout 20 -- -d $O:serialcomm=115200/8n1 --scan
    emu_options uart=115200/8n1
    oxla_case "serialcomm as the UART" --keep-header -- -d $O:serialcomm=115200/8n1 --scan
    emu_options

    # Each option.
    for key in conn samplerate captureratio limit_samples pattern rle; do
        oxla_case "get $key" -- -d $O --get $key
    done
    for setting in samplerate=135m samplerate=108m samplerate=81m samplerate=67500k \
        samplerate=27m samplerate=10m samplerate=1m samplerate=1k samplerate=5 \
        captureratio=50 captureratio=100 limit_samples=1 limit_samples=4194304 \
        pattern=Internal pattern=None rle=on rle=off; do
        key=${setting%%=*}
        oxla_case "set $setting" -- -d $O -c $setting --get $key
    done
    oxla_case "set samplerate too high" --stderr --expect-fail -- -d $O -c samplerate=136m --set
    oxla_case "set samplerate too low" --stderr --expect-fail -- -d $O -c samplerate=4 --set
    oxla_case "set samplerate 0" --stderr --expect-fail -- -d $O -c samplerate=0 --set
    oxla_case "set limit_samples 0" --stderr --expect-fail -- -d $O -c limit_samples=0 --set
    oxla_case "set captureratio 101" --stderr --expect-fail -- -d $O -c captureratio=101 --set
    oxla_case "set pattern unknown" --stderr --expect-fail -- -d $O -c pattern=Bogus --set
    emu_options channels=14 spc=2
    oxla_case "get samplerate la14-ddr" -- -d $O --get samplerate
    oxla_case "set samplerate 216m" -- -d $O -c samplerate=216m --get samplerate
    oxla_case "set samplerate 200m" -- -d $O -c samplerate=200m --get samplerate
    oxla_case "set rle without RLE" --stderr --expect-fail -- -d $O -c rle=on --set
    oxla_case "get rle without RLE" --stderr --expect-fail -- -d $O --get rle
    emu_options

    # Captures of the counter. What differs between them is on the wire:
    # the clock, the divider, the count of samples, the words read.
    for n in 1 2 3 8 100 1000; do
        oxla_case "samples $n" -- -d $O $P --samples $n -O hex
    done
    oxla_case "samples 16 bits" -- -d $O $P --samples 16 -O bits
    oxla_case "samples 16 csv" -- -d $O $P --samples 16 $CSV
    # Three requests of memory, the last a part of one.
    oxla_case "samples 10000" --digest --filter '$=; d' -- -d $O $P --samples 10000 -O hex
    oxla_case "samples beyond the memory" --stderr --expect-fail -- -d $O $P --samples 4194305 -O hex
    for rate in 135m 108m 81m 54m 27m 10m 1m 1k; do
        oxla_case "rate $rate" -- -d $O $P -c samplerate=$rate --samples 8 -O hex
    done
    # The pins: the count of clocks, so a divided rate skips.
    oxla_case "pins" -- -d $O --samples 16 -O hex
    oxla_case "pins 27m" -- -d $O -c samplerate=27m --samples 16 -O hex
    oxla_case "pins 81m" -- -d $O -c samplerate=81m --samples 16 -O hex
    # Disabled channels are low, in the counter too.
    oxla_case "channels" -- -d $O $P -C D4,D5,D6,D7 --samples 32 -O hex
    oxla_case "channels pins" -- -d $O -C D0,D15 --samples 32 -O hex
    emu_options channels=14 spc=2
    oxla_case "la14-ddr samples 16" -- -d $O $P --samples 16 -O hex
    # Samples that straddle words, and a last word partly filled.
    for n in 1 2 3 7 100 1001; do
        oxla_case "la14-ddr samples $n" -- -d $O $P --samples $n -O bits
    done
    oxla_case "la14-ddr 135m" -- -d $O $P -c samplerate=135m --samples 16 -O hex
    oxla_case "la14-ddr pins" -- -d $O --samples 16 -O hex
    oxla_case "la14-ddr rle" --stderr --expect-fail -- -d $O -c rle=on --samples 8 -O hex
    emu_options channels=8 spc=2
    oxla_case "la8-ddr samples 16" -- -d $O $P --samples 16 -O hex
    oxla_case "la8-ddr samples 1001" --digest --filter '$=; d' -- -d $O $P --samples 1001 -O hex
    emu_options channels=8 rle=0 adc=1
    oxla_case "la8-adc samples 16" -- -d $O $P --samples 16 -O hex
    emu_options

    # Triggers. The counter's bit n rises at 2^n.
    oxla_case "trigger" -- -d $O $P --samples 16 -t D5=r $CSV
    oxla_case "trigger ratio 50" -- -d $O $P -c captureratio=50 --samples 16 -t D5=r $CSV
    # It comes before the share ahead of it is there: a shorter capture.
    oxla_case "trigger early" -- -d $O $P -c captureratio=50 --samples 32 -t D2=r $CSV
    oxla_case "trigger ratio 100" -- -d $O $P -c captureratio=100 --samples 16 -t D5=r $CSV
    oxla_case "trigger levels" -- -d $O $P -c captureratio=50 --samples 16 -t D0=1,D1=0,D2=1,D3=1 $CSV
    oxla_case "trigger falling" -- -d $O $P -c captureratio=50 --samples 16 -t D3=f $CSV
    oxla_case "trigger either edge" -- -d $O $P -c captureratio=50 --samples 16 -t D4=e $CSV
    oxla_case "trigger late" -- -d $O $P -c captureratio=50 --samples 16 -t D12=r $CSV
    oxla_case "trigger two stages" -- -d $O $P -c captureratio=50 --samples 16 -t 'D5=r;D3=f' $CSV
    oxla_case "trigger two letters" -- -d $O $P -c captureratio=50 --samples 16 -t D5=rf $CSV
    oxla_case "trigger or" -- -d $O $P -c captureratio=50 --samples 16 -t 'D7=r|D5=f' $CSV
    oxla_case "trigger count" -- -d $O $P -c captureratio=50 --samples 16 -t 'D2=e*1000' $CSV
    oxla_case "trigger and" --stderr --expect-fail -- -d $O $P --samples 16 -t 'D7=r&D5=f' $CSV
    oxla_case "trigger three stages" --stderr --expect-fail -- -d $O $P --samples 16 -t 'D1=r;D2=r;D3=r' $CSV
    oxla_case "trigger count on two stages" --stderr --expect-fail -- -d $O $P --samples 16 -t 'D1=r*2;D2=r' $CSV
    oxla_case "trigger consecutive count" --stderr --expect-fail -- -d $O $P --samples 16 -t 'D1=r*2c' $CSV
    oxla_case "trigger duration" --stderr --expect-fail -- -d $O $P --samples 16 -t 'D1=1>10' $CSV
    oxla_case "trigger divided rate" -- -d $O $P -c samplerate=27m:captureratio=50 --samples 16 -t D5=r $CSV
    oxla_case "trigger pins" -- -d $O -c samplerate=27m:captureratio=50 --samples 16 -t D5=r $CSV
    emu_options channels=14 spc=2
    oxla_case "la14-ddr trigger" -- -d $O $P -c captureratio=50 --samples 16 -t D13=r $CSV
    for at in D0=1 D1=1,D0=0 D1=1,D0=1 D2=1,D1=0,D0=0 D2=1,D1=0,D0=1; do
        oxla_case "la14-ddr trigger $at" -- -d $O $P --samples 4 -t $at $CSV
    done
    oxla_case "la14-ddr trigger beyond the channels" --stderr --expect-fail -- -d $O $P --samples 16 -t D14=r $CSV
    emu_options channels=8 spc=2
    oxla_case "la8-ddr trigger" -- -d $O $P -c captureratio=50 --samples 16 -t D7=f $CSV
    # A ring of 70 words written around 26 times before the trigger.
    emu_options channels=14 spc=2 mem=70
    oxla_case "ring around" -- -d $O $P -c captureratio=60 --samples 150 -t D12=r $CSV
    oxla_case "ring holds 160" --stderr --expect-fail -- -d $O $P --samples 161 -O hex
    emu_options

    # Run length encoding. The counter is a word a sample; the pins held
    # for a while are runs.
    oxla_case "rle" -- -d $O $P -c rle=on:samplerate=54m --samples 16 -O hex
    oxla_case "rle trigger" -- -d $O $P -c rle=on:samplerate=54m:captureratio=50 --samples 16 -t D5=r $CSV
    oxla_case "rle beyond the raw memory" -- -d $O -c rle=on:limit_samples=4194305 --get limit_samples
    emu_options slow=1000
    oxla_case "rle runs" --digest --filter '$=; d' -- -d $O -c rle=on --samples 5000 -O hex
    oxla_case "rle runs trigger" -- -d $O -c rle=on:captureratio=50 --samples 16 -t D1=r $CSV
    oxla_case "rle runs cut inside a run" -- -d $O -c rle=on:captureratio=50 --samples 1500 -t D1=r $CSV
    # Runs longer than their count holds: 65536 a word.
    emu_options slow=100000
    oxla_case "rle long runs" --digest --filter '$=; d' -- -d $O -c rle=on --samples 300000 -O hex
    emu_options channels=8 slow=1000
    oxla_case "la8-rle runs" --digest --filter '$=; d' -- -d $O -c rle=on --samples 5000 -O hex
    emu_options

    # What the device does wrong.
    # A word a sample at the full rate is more than the memory takes.
    oxla_case "rle overflow" --oxrok-fails "the sample memory overflowed" --no-output -- -d $O $P -c rle=on --samples 100000 -O hex
    emu_options overflow=20
    oxla_case "overflow" --oxrok-fails "the sample memory overflowed" --no-output -- -d $O $P --samples 1000 -O hex
    # A byte of the second request lost on the way.
    emu_options lose=9000
    oxla_case "a byte lost" --oxrok-fails "the memory read went silent" --no-output --timeout 20 -- -d $O $P --samples 10000 -O hex
    emu_options polls=5
    oxla_case "five looks at the status" -- -d $O $P --samples 8 -O hex
    emu_options settle=3
    oxla_case "clock ready late" -- -d $O $P -c samplerate=81m --samples 8 -O hex
    emu_options chunk=1
    oxla_case "one byte per transfer" -- -d $O $P --samples 16 -O hex
    emu_options

    # A trigger that never comes: a counter's bit 1 does not rise with bit
    # 0. A stop sends the device's STOP and what it had, the last samples
    # of the model's search, with no marker; how many looks at the status
    # came before the signal is the pace's, so the transactions end there.
    oxla_case "interrupt waiting" --interrupt model --timeout 30 --tx-filter '/^interrupt$/q' -- -d $O $P --samples 16 -t D0=r,D1=r $CSV
    # An unplug while the device waits for its trigger. One in the scan
    # ends in no device or in an error as the pace has it: a unit test has
    # the device that goes silent.
    oxla_case "unplug waiting" --oxrok-fails "the port gone" --no-output --unplug model --timeout 20 --tx-filter '/^unplug$/q' -- -d $O $P --samples 16 -t D0=r,D1=r -O hex
}

# See emu_options in tools/diffcli.sh.
emu_conns() {
    O=oxla:conn=$EMU_TTY
}
