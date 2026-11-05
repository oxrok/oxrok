# A Link Instruments MSO-19 (emu/src/models/link-mso19.rs) on the CDC-ACM shell.
# Sourced by tools/diffcli.sh --emu; see docs/USB.md, "Writing a serial model".
EMU_MODEL=link-mso19
EMU_CDC=1
EMU_DRIVER=link-mso19
EMU_FULL=$EMU_DRIVER
EMU_GCOV=link-mso19
EMU_GCOV_SHARED="src/serial.c src/serial_libsp.c"

emu_cases() {
    emu_conns
    # A capture's output is 1024 samples: the first lines and the digest of it all.
    D='3,$d'
    # A poll in a capture and what answers it, whose count is the time.
    T='/^bulk_out 02 404c44537e42407e -> 8$/d
/^bulk_in 82 1 -> 34$/d
/^interrupt$/d'
    # The core's line about a refused start holds a pointer.
    P='s/(non-existing event source) 0x[0-9a-f]+/\1/'
    # oxrok's log prefix, on its side only.
    L='s/^oxrok: [a-z]+: oxrok::drivers::link_mso19(::protocol)?: /sr: link-mso19: /'
    # A refused setting or start in each tool's words: the core's lines are C's
    # alone, and what follows is the error each has for it.
    A='s/^(Failed to set device option .[a-z_]+.: invalid argument).*$/\1./
s/^(Failed to set device option .[a-z_]+.: )device error: generic\/unspecified error\.$/\1generic\/unspecified error./
s/^Failed to set channels: invalid argument: //'
    # Each tool's way of saying that an open failed: the driver's line comes first.
    O="$L
s/^(Failed to open device): .*$/\1./"
    S="$L
$P
/^sr: session: /d
s/^(Failed to start session).*$/\1./
s/^device error: .*$/Failed to start session./"
    THRESHOLD_BUG="protocol.c:90-93, since 932a3452, hold the sum in an int and divide it by the probe factor as an int, so 15 V on a 10x probe is 1 V and 9 V is 0 V, while the level reads back as set; the commit before it, and mso19fcgi, divide the voltage, and oxrok does"
    READY_BUG="api.c:647 keeps the start's status as devc->status, so protocol.c:390 takes a READY there for a dump asked for and copies the next status reply into the buffer; no request goes out and the session waits for ever; oxrok asks again, and dumps"
    SILENT_BUG="serial_source_add's timeout of -1 (api.c:659) and no other bound: a device that stops answering holds C for ever; oxrok ends after 5 s"
    WRAP_BUG="api.c:530 stores a uint64 in a uint16_t after refusing 0, so 65537 is 1 with exit 0; oxrok refuses what does not fit"
    FPE_BUG="the same store makes 65536 a 0, and protocol.c:93 then divides an int by it: SIGFPE at the start; oxrok refuses it at the set"
    NULL_BUG="sp_get_port_usb_serial gives null for a device with no serial number, and protocol.c:210 reads serial_num[0]; oxrok says the serial is invalid"
    RATE_BUG="mso_configure_rate takes a uint32_t (protocol.c:290), so 2^32 + 100 is 100 Hz with exit 0; oxrok refuses it"
    NAN_BUG="api.c:510's two comparisons let a position that is no number through, to a register of 0; oxrok refuses it"
    FLOAT_DIFF="sigrok-cli parses a float in single precision, so 3.3 is 3.2999999523, under 55 steps of the DAC where the double is over: one step lower; a library client of libsigrok has what oxrok has"
    CSV_BUG="csv.c:500 indexes the logic samples by the column, which counts DSO, so each logic column shows the next channel's value, and the last the next sample's channel 0, reading past the buffer on the last line (which is why that line is cut here); oxrok has the right channel"
    LOOSE_BUG="sigrok-cli takes what is no number as 0 (strtod, unchecked) and sets it; oxrok refuses it and sends nothing"
    REFUSED_TRIGGER="sigrok-cli exits 0 after sr_trigger_match_add refuses the match, capturing nothing; oxrok fails"
    SIGINT_BUG="sigrok-cli has no SIGINT handler, it dies where oxrok ends the capture"

    # A scan opens nothing. The serial number holds the calibration.
    emu_case "scan" --keep-header -- -d $K --scan
    emu_case "scan with no conn" --keep-header -- -d link-mso19 --scan
    emu_case "no such node" --keep-header --no-output -- -d link-mso19:conn=/dev/ttyACM97 --scan
    emu_case "conn that is no USB port" --keep-header --no-output -- -d link-mso19:conn=/dev/null --scan
    emu_options pid=1
    emu_case "another product" --keep-header --no-output -- -d $K --scan
    emu_case "another product with no conn" --keep-header --no-output -- -d link-mso19 --scan
    emu_options vid=1
    emu_case "another vendor" --keep-header --no-output -- -d $K --scan
    emu_options manufacturer=Maker product=Scope serial=4287434350019000123
    emu_case "strings and revision 9" --keep-header -- -d $K --scan
    emu_options manufacturer= product=
    emu_case "no strings" --keep-header -- -d $K --scan
    # The fields of the serial number, none of them checked but the first digit.
    emu_options serial=3287434350012000123
    emu_case "serial starts with no 4" --stderr --filter "$L" -- -d $K --scan
    emu_options serial=4287434350012
    emu_case "serial with no unit number" --stderr --filter "$L" -- -d $K --scan
    emu_options serial=x
    emu_case "serial no digits" --stderr --filter "$L" -- -d $K --scan
    emu_options serial=42874343500120
    emu_case "serial of one unit digit" --keep-header -- -d $K --scan
    emu_options serial=4xxxx343500120001
    emu_case "serial with letters" --stderr --filter "$L" -- -d $K --scan
    # With no serial number mso_parse_serial reads through the null it gets.
    emu_options serial=
    emu_case "no serial number" --oracle-bug "$NULL_BUG" --stderr --filter "$L" -- -d $K --scan
    emu_options serial=4287434350012000123extra
    emu_case "serial longer" --keep-header -- -d $K --scan
    emu_options
    emu_case "show" --keep-header -- -d $K --show
    emu_case "show DSO" --keep-header -- -d $K -g DSO --show
    emu_case "show LA" --keep-header -- -d $K -g LA --show
    emu_case "show no such group" --stderr --expect-fail -- -d $K -g XX --show

    # Each get.
    for key in samplerate limit_samples triggersource horiz_triggerpos; do
        emu_case "get $key" -- -d $K --get $key
    done
    for key in coupling triggerslope triggerlevel probe_factor offset; do
        emu_case "get DSO $key" -- -d $K -g DSO --get $key
    done
    for key in triggerslope logic_threshold; do
        emu_case "get LA $key" -- -d $K -g LA --get $key
    done
    emu_case "get coupling with no group" --stderr --expect-fail -- -d $K --get coupling
    emu_case "get coupling on LA" --stderr --expect-fail -- -d $K -g LA --get coupling
    emu_case "get threshold on DSO" --stderr --expect-fail -- -d $K -g DSO --get logic_threshold

    # A rate goes out at once: every line of the table.
    for r in 200m 100m 50m 20m 10m 5m 2m 1m 500k 200k 100k 50k 20k 10k 5k 2k 1k 500 200 100; do
        emu_case "set rate $r" --no-output -- -d $K -c samplerate=$r --set
    done
    emu_case "set rate not in the table" --stderr --filter "$L
$A" --expect-fail -- -d $K -c samplerate=3m --set
    emu_case "set then get rate" -- -d $K -c samplerate=1m --get samplerate
    emu_case "set limit 1024" --no-output -- -d $K -c limit_samples=1024 --set
    emu_case "set limit 1023" --stderr --filter "$L
$A" --expect-fail -- -d $K -c limit_samples=1023 --set
    emu_case "samples 1023" --stderr --filter "$L
$A" --expect-fail -- -d $K --samples 1023
    emu_case "set no such source" --stderr --filter "$A" --expect-fail -- -d $K -c triggersource=SPI --set
    emu_case "set then get source" -- -d $K -c triggersource=LA --get triggersource
    emu_case "set then get DSO slope" -- -d $K -g DSO -c triggerslope=Falling --get triggerslope
    emu_case "set then get LA slope" -- -d $K -g LA -c triggerslope=T-\>F --get triggerslope
    emu_case "set DSO slope of LA's" --stderr --filter "$A" --expect-fail -- -d $K -g DSO -c triggerslope=T-\>F --set
    emu_case "set LA slope of DSO's" --stderr --filter "$A" --expect-fail -- -d $K -g LA -c triggerslope=Rising --set
    emu_case "set then get coupling" -- -d $K -g DSO -c coupling=AC --get coupling
    emu_case "set no such coupling" --stderr --filter "$A" --expect-fail -- -d $K -g DSO -c coupling=GND --set
    emu_case "set then get threshold" -- -d $K -g LA -c logic_threshold=1.8V\ Logic --get logic_threshold
    emu_case "set no such threshold" --stderr --filter "$A" --expect-fail -- -d $K -g LA -c logic_threshold=5V --set
    emu_case "set then get probe" -- -d $K -g DSO -c probe_factor=100 --get probe_factor
    emu_case "set probe 0" --stderr --filter "$A" --expect-fail -- -d $K -g DSO -c probe_factor=0 --set
    emu_case "probe 65537 wraps" --oracle-differs "$WRAP_BUG" --expect-fail --stderr --filter "$A" -- -d $K -g DSO -c probe_factor=65537 --get probe_factor
    emu_case "set then get level" -- -d $K -g DSO -c triggerlevel=1.5 --get triggerlevel
    emu_case "set then get level too high" -- -d $K -g DSO -c triggerlevel=1000 --get triggerlevel
    emu_case "set then get offset" -- -d $K -g DSO -c offset=1.5 --get offset
    emu_case "set then get offset too high" -- -d $K -g DSO -c offset=1000 --get offset
    emu_case "level is no float" --oracle-differs "$LOOSE_BUG" --expect-fail --stderr --filter "$A" -- -d $K -g DSO -c triggerlevel=x --set
    emu_case "set then get position" -- -d $K -c horiz_triggerpos=0.25 --get horiz_triggerpos
    emu_case "position at the top" -- -d $K -c horiz_triggerpos=1 --get horiz_triggerpos
    emu_case "position at the bottom" -- -d $K -c horiz_triggerpos=-10 --get horiz_triggerpos
    emu_case "position too high" --stderr --filter "$L
$A" --expect-fail -- -d $K -c horiz_triggerpos=1.5 --set
    emu_case "position too low" --stderr --filter "$L
$A" --expect-fail -- -d $K -c horiz_triggerpos=-10.5 --set
    emu_case "position warns under .01" --stderr --filter "$L" -- -d $K -c horiz_triggerpos=0.001 --set
    emu_case "position warns over -1.011" --stderr --filter "$L" -- -d $K -c horiz_triggerpos=-0.5 --set
    emu_case "position over no warning" --stderr --filter "$L" --no-output -- -d $K -c horiz_triggerpos=-2 --set

    # Captures.
    emu_case "capture" --digest --filter "$D" -- -d $K --samples 1024
    emu_case "capture analog" --digest --filter "$D" -- -d $K --samples 1024 -O analog
    emu_case "capture csv" --oracle-differs "$CSV_BUG" --filter '1024,$d' -- -d $K --samples 1024 -O csv:header=false:time=false
    emu_case "capture csv trigger" --oracle-differs "$CSV_BUG" --filter "$D" -- -d $K --samples 1024 -O csv:header=false:time=false:trigger=true
    emu_case "capture hex" --digest --filter "$D" -- -d $K --samples 1024 -O hex
    emu_case "capture bits is all of it" -- -d $K --samples 1024
    emu_case "time at the rate" --digest --filter "$D" -- -d $K -c samplerate=100 --time 10240
    emu_case "time under the memory" --stderr --filter "$L" --expect-fail -- -d $K --time 10
    emu_case "continuous" --stderr --expect-fail -- -d $K --continuous
    emu_case "frames" --stderr --expect-fail -- -d $K --frames 1
    emu_case "logic only" --digest --filter "$D" -- -d $K -C 0,1,2,3,4,5,6,7 --samples 1024
    emu_case "analog only" --digest --filter "$D" -- -d $K -C DSO --samples 1024 -O analog
    emu_case "logic 4 alone" --digest --filter "$D" -- -d $K -C 4 --samples 1024
    emu_case "no such channel" --stderr --filter "$A" --expect-fail -- -d $K -C 8 --samples 1024
    emu_case "rename" --digest --filter "$D" -- -d $K -C 0=A,DSO=V --samples 1024 -O analog
    # The registers of a start.
    emu_case "capture slow rate" --digest --filter "$D" -- -d $K -c samplerate=100 --samples 1024
    emu_case "capture fast rate" --digest --filter "$D" -- -d $K -c samplerate=200m --samples 1024
    emu_case "capture AC" --digest --filter "$D" -- -d $K -g DSO -c coupling=AC --samples 1024
    emu_case "capture falling" --digest --filter "$D" -- -d $K -g DSO -c triggerslope=Falling --samples 1024
    emu_case "capture on LA" --digest --filter "$D" -- -d $K -c triggersource=LA --samples 1024
    emu_case "capture on LA T-F" --digest --filter "$D" -- -d $K -c triggersource=LA -c channel_group=LA:triggerslope=T-\>F --samples 1024
    emu_case "capture threshold" --digest --filter "$D" -- -d $K -g LA -c logic_threshold=1.2V\ Logic --samples 1024
    emu_case "capture level 1.5" --oracle-differs "$THRESHOLD_BUG" --digest --filter "$D" -- -d $K -g DSO -c triggerlevel=1.5 --samples 1024
    emu_case "capture level 15" --oracle-differs "$THRESHOLD_BUG" --digest --filter "$D" -- -d $K -g DSO -c triggerlevel=15 --samples 1024
    emu_case "capture level minus 15" --oracle-differs "$THRESHOLD_BUG" --digest --filter "$D" -- -d $K -g DSO -c triggerlevel=-15 --samples 1024
    emu_case "capture level clamped" --digest --filter "$D" -- -d $K -g DSO -c triggerlevel=1000 --samples 1024
    emu_case "capture level clamped low" --digest --filter "$D" -- -d $K -g DSO -c triggerlevel=-1000 --samples 1024
    emu_case "capture offset 3" --oracle-differs "$THRESHOLD_BUG" --digest --filter "$D" -- -d $K -g DSO -c offset=3 --samples 1024
    emu_case "capture offset clamped" --oracle-differs "$THRESHOLD_BUG" --digest --filter "$D" -- -d $K -g DSO -c offset=-1000 --samples 1024
    emu_case "capture offset and level" --oracle-differs "$THRESHOLD_BUG" --digest --filter "$D" -- -d $K -g DSO -c offset=3.3 -c channel_group=DSO:triggerlevel=12.5 --samples 1024
    emu_case "capture probe 1" --oracle-differs "$THRESHOLD_BUG" --digest --filter "$D" -- -d $K -g DSO -c probe_factor=1 -c channel_group=DSO:triggerlevel=1.5 --samples 1024
    emu_case "capture probe wraps to 0" --oracle-bug "$FPE_BUG" --stderr --filter "$A" --expect-fail -- -d $K -g DSO -c probe_factor=65536 --samples 1024
    emu_case "capture position 0" --digest --filter "$D" -- -d $K -C 0,1,2,3,4,5,6,7 -c horiz_triggerpos=0 --samples 1024 -O csv:header=false:time=false:trigger=true
    emu_case "capture position 1" --digest --filter "$D" -- -d $K -C 0,1,2,3,4,5,6,7 -c horiz_triggerpos=1 --samples 1024 -O csv:header=false:time=false:trigger=true
    emu_case "capture position 0.25" --digest --filter "$D" -- -d $K -C 0,1,2,3,4,5,6,7 -c horiz_triggerpos=0.25 --samples 1024 -O csv:header=false:time=false:trigger=true
    emu_case "capture holdoff" --digest --filter "$D" -- -d $K -C 0,1,2,3,4,5,6,7 -c horiz_triggerpos=-2.5 --samples 1024 -O csv:header=false:time=false:trigger=true
    emu_case "capture holdoff rounds to 0" --digest --filter "$D" -- -d $K -C 0,1,2,3,4,5,6,7 -c horiz_triggerpos=-0.0001 --samples 1024 -O csv:header=false:time=false:trigger=true
    # A level off the whole volts, on the double's side of the DAC's step.
    emu_case "capture offset 3.3" --oracle-differs "$FLOAT_DIFF" --digest --filter "$D" -- -d $K -g DSO -c offset=3.3 --samples 1024
    # Triggers.
    emu_case "trigger" --digest --filter "$D" -- -d $K -t 0=1,3=0,7=1 --samples 1024
    emu_case "trigger on a disabled channel" --stderr --expect-fail -- -d $K -C 0,1 -t 0=1,5=1 --samples 1024
    emu_case "trigger no such match" --stderr --expect-fail -- -d $K -t 0=r --samples 1024
    emu_case "trigger on DSO" --oracle-differs "$REFUSED_TRIGGER" --expect-fail --stderr -- -d $K -t DSO=1 --samples 1024
    emu_case "trigger on LA source" --digest --filter "$D" -- -d $K -c triggersource=LA -t 4=1 --samples 1024
    # Calibration from the serial number shows in a capture's registers and values.
    emu_options serial=4000002000012000123
    emu_case "capture serial of defaults" --oracle-differs "$THRESHOLD_BUG" --digest --filter "$D" -- -d $K -g DSO -c offset=1:triggerlevel=15 --samples 1024 -O analog
    emu_options serial=4287400000012000123
    emu_case "capture serial with zero fields" --oracle-differs "$THRESHOLD_BUG" --digest --filter "$D" -- -d $K -g DSO -c offset=1:triggerlevel=15 --samples 1024 -O analog
    emu_options serial=4123450060012000123
    emu_case "capture another calibration" --oracle-differs "$THRESHOLD_BUG" --digest --filter "$D" -- -d $K -g DSO -c offset=1:triggerlevel=15 --samples 1024 -O analog
    emu_options serial=41234-5060012000123
    emu_case "capture a sign in the serial" --digest --filter "$D" -- -d $K -g DSO -c offset=1:triggerlevel=15 --samples 1024 -O analog
    emu_options
    # How the dump comes.
    emu_options chunk=1
    emu_case "capture a byte per transfer" --digest --filter "$D" -- -d $K --samples 1024
    emu_options pause=1500:3
    emu_case "capture dump pauses" --digest --filter "$D" -- -d $K --samples 1024
    emu_options extra=100
    emu_case "capture dump with more" --digest --filter "$D" -- -d $K --samples 1024
    emu_options pre=5 wait=20 post=5
    emu_case "capture many polls" --digest --filter "$D" -- -d $K --samples 1024
    emu_options

    # Status faults.
    emu_options status-at=1:ff
    emu_case "open status invalid" --stderr --filter "$O" --expect-fail -- -d $K --show
    emu_options status-at=1:00
    emu_case "open status 0" --stderr --filter "$O" --expect-fail -- -d $K --show
    emu_options status-at=1:07
    emu_case "open status 7" --stderr --filter "$O" --expect-fail -- -d $K --show
    emu_options mute-at=1
    emu_case "open status silent" --stderr --filter "$O" --expect-fail -- -d $K --show
    emu_options status-at=2:7f
    emu_case "start status invalid" --stderr --filter "$S" --expect-fail -- -d $K --samples 1024
    emu_options mute-at=2
    emu_case "start status silent" --stderr --filter "$S" --expect-fail -- -d $K --samples 1024
    emu_options status-at=3:ff
    emu_case "poll status invalid" --stderr --filter "$L" -- -d $K --samples 1024
    emu_options late-at=4:5
    emu_case "poll status late" --digest --filter "$D" -- -d $K --samples 1024
    # Never ends, as what it waits for does not come.
    emu_options mute-at=3
    emu_case "poll status silent" --oracle-bug "$SILENT_BUG" --oracle-hangs --no-output --timeout 8 -- -d $K --samples 1024
    emu_options dump=100
    emu_case "dump short" --oracle-bug "$SILENT_BUG" --oracle-hangs --no-output --timeout 8 -- -d $K --samples 1024
    emu_options raw=1
    emu_case "dump with flow control bytes" --oracle-bug "$SILENT_BUG" --oracle-hangs --no-output --timeout 8 -- -d $K --samples 1024
    emu_options pre=0 wait=0 post=0
    emu_case "ready at the start" --oracle-bug "$READY_BUG" --oracle-hangs --digest --filter "$D" --timeout 3 -- -d $K --samples 1024
    # A trigger that never fires is no error: both wait, polling. The stop is on
    # the mark, a few polls in, and how many more go by depends on when it lands.
    emu_options wait=never
    emu_case "lib trigger never fires" --libsigrok --hangs --no-output --timeout 2 --tx-filter "$T" -- -d $K --samples 1024
    emu_options wait=never mark-at=6
    emu_case "trigger never fires, interrupted" --oracle-bug "$SIGINT_BUG" --no-output --interrupt model --timeout 15 --tx-filter "$T" -- -d $K --samples 1024

    # Unplugs.
    emu_options mute-at=3
    emu_case "unplug waiting for a status" --no-output --unplug model --timeout 3 -- -d $K --samples 1024
    emu_options dump=100
    emu_case "unplug in the dump" --no-output --unplug model --timeout 3 -- -d $K --samples 1024
    emu_options stall=line
    emu_case "line coding stalled" --digest --filter "$D" -- -d $K --samples 1024
    emu_options stall=lines
    emu_case "control lines stalled" --digest --filter "$D" -- -d $K --samples 1024
    emu_options

    # What sigrok-cli cannot ask for.
    emu_options wait=never
    emu_case "lib stop" --libsigrok --no-output -- -d $K --samples 1024 --stop-after 0
    emu_options
}

# See emu_options in tools/diffcli.sh.
emu_conns() {
    K=link-mso19:conn=$EMU_TTY
}
