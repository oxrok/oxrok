# RDTech DPS, DPH and RD power supplies (emu/src/models/rdtech-dps.rs), a
# Modbus RTU slave behind a UART, on the CDC-ACM shell. Sourced by
# tools/diffcli.sh --emu; see docs/USB.md, "Writing a serial model".
EMU_MODEL=rdtech-dps
EMU_CDC=1
EMU_DRIVER="rdtech-dps rdtech-rd"
EMU_GCOV=rdtech-dps
EMU_GCOV_SHARED="src/serial.c src/serial_libsp.c src/modbus/modbus.c src/modbus/modbus_serial_rtu.c src/crc.c"

emu_cases() {
    emu_conns
    # Every line of api.c's supported_models[]. A scan reads the model and
    # the version, an RD with more than one range its range as well, and
    # opens nothing else.
    for id in 3005 5005 5205 5015 5020 8005; do
        emu_options setters=0 id=$id
        emu_case "scan DPS $id" --keep-header -- -d $D --scan
    done
    for id in 60061 60062 60065 60121 60125 60181 60241; do
        emu_options setters=0 id=$id
        emu_case "scan RD $id" --driver rdtech-rd --keep-header -- -d $R --scan
    done
    # How the rest of the identity shows.
    emu_options setters=0 id=60061 serial=0
    emu_case "scan RD no serial" --driver rdtech-rd --keep-header -- -d $R --scan
    emu_options setters=0 id=60061 version=5 serial=4294967295
    emu_case "scan RD version and serial corners" --driver rdtech-rd --keep-header -- -d $R --scan
    emu_options setters=0 id=9999
    emu_case "scan DPS unknown model" --stderr --keep-header -- -d $D --scan
    emu_options setters=0 id=60999
    emu_case "scan RD unknown model" --driver rdtech-rd --stderr --keep-header -- -d $R --scan
    # The other family's driver on a supply: its map says something else.
    emu_options setters=0 id=60125
    emu_case "scan DPS driver on an RD" --stderr --keep-header -- -d $D:serialcomm=115200/8n1 --scan
    emu_options setters=0
    emu_case "scan RD driver on a DPS" --driver rdtech-rd --stderr --keep-header -- -d $R:serialcomm=9600/8n1 --scan
    emu_case "no conn" --keep-header --no-output -- -d rdtech-dps --scan
    emu_case "no such node" --keep-header --no-output -- -d rdtech-dps:conn=/dev/ttyACM97 --scan

    # The line and the address.
    emu_options setters=0
    emu_case "RD driver misses a DPS's line" --driver rdtech-rd --keep-header --no-output -- -d $R --scan
    emu_case "serialcomm misset" --keep-header --no-output -- -d $D:serialcomm=19200/8n1 --scan
    emu_case "serialcomm as it is" --keep-header -- -d $D:serialcomm=9600/8n1 --scan
    emu_options setters=0 uart=19200/8n1
    emu_case "serialcomm on a changed line" --keep-header -- -d $D:serialcomm=19200/8n1 --scan
    emu_options setters=0 addr=5
    emu_case "modbusaddr of the slave" --keep-header -- -d $D:modbusaddr=5 --scan
    emu_case "modbusaddr default misses it" --keep-header --no-output -- -d $D --scan
    emu_options setters=0
    emu_case "modbusaddr other" --keep-header --no-output -- -d $D:modbusaddr=7 --scan
    emu_case "modbusaddr 0 is a broadcast" --keep-header --no-output -- -d $D:modbusaddr=0 --scan
    emu_case "modbusaddr over 8 bits" --keep-header -- -d $D:modbusaddr=257 --scan

    # What a slave can answer in the probe. A read is tried three times.
    for code in 01 02 03 04 05 06 08 0a 0b 42; do
        emu_options setters=0 faults=1:exc$code,2:exc$code,3:exc$code
        emu_case "scan exception $code" --stderr --keep-header -- -d $D --scan
    done
    for kind in silent crc addr short count func; do
        emu_options setters=0 faults=1:$kind
        case $kind in
            # What is left of another slave's frame fails the retries as well.
            addr) emu_case "scan one $kind" --keep-header --no-output -- -d $D --scan ;;
            *) emu_case "scan one $kind, then fine" --keep-header -- -d $D --scan ;;
        esac
        emu_options setters=0 faults=1:$kind,2:$kind,3:$kind
        case $kind in
            crc | short) emu_case "scan three $kind" --stderr --keep-header -- -d $D --scan ;;
            *) emu_case "scan three $kind" --keep-header --no-output -- -d $D --scan ;;
        esac
    done
    emu_options setters=0 faults=1:extra
    emu_case "scan two bytes too many" --keep-header -- -d $D --scan
    # The range's read, an RD6012P's second step.
    emu_options setters=0 id=60125 faults=2:silent,3:silent,4:silent
    emu_case "scan the range read fails" --driver rdtech-rd --keep-header --no-output -- -d $R --scan

    # --show and --get, each read being all of the state.
    emu_options setters=0 id=60125
    emu_case "show RD6012P" --driver rdtech-rd --keep-header -- -d $R --show
    emu_options setters=0 id=60125 range=1
    emu_case "show RD6012P 12A" --driver rdtech-rd --keep-header -- -d $R --show
    emu_options setters=0 id=60125 output=1 load=10 uset=12000 iset=20000 protect=1
    emu_case "show RD6012P on" --driver rdtech-rd --keep-header -- -d $R --show
    for key in voltage voltage_target current current_limit enabled regulation ovp_active \
        ovp_threshold ocp_active ocp_threshold range limit_samples limit_time; do
        emu_options setters=0 id=60125 output=1 load=1 uset=12000 iset=20000 protect=2
        emu_case "get RD6012P $key" --driver rdtech-rd -- -d $R --get $key
    done
    emu_options setters=0 id=60125 range=1 iset=1500
    emu_case "get RD6012P current_limit 12A" --driver rdtech-rd -- -d $R --get current_limit
    emu_case "get RD6012P range 12A" --driver rdtech-rd -- -d $R --get range
    emu_options setters=0 id=60125 protect=1
    emu_case "get RD6012P ovp_active on OVP" --driver rdtech-rd -- -d $R --get ovp_active
    emu_options setters=0 id=60125 protect=3
    emu_case "get RD6012P ovp_active on OPP" --driver rdtech-rd -- -d $R --get ovp_active
    emu_options setters=0 id=60125
    emu_case "get RD6012P unlisted ovp_enabled" --driver rdtech-rd --stderr --expect-fail -- -d $R --get ovp_enabled
    emu_case "get RD6012P conn" --driver rdtech-rd --stderr --expect-fail -- -d $R --get conn
    # One range, where libsigrok never sets its multipliers.
    emu_options setters=0 id=5005
    emu_case "show DPS5005" --keep-header -- -d $D --show
    emu_options setters=0 id=5020
    emu_case "show DPS5020" --keep-header -- -d $D --show
    emu_options setters=0 id=60061
    emu_case "show RD6006" --driver rdtech-rd --keep-header -- -d $R --show
    emu_options setters=0 id=60065
    emu_case "show RD6006P" --driver rdtech-rd --keep-header -- -d $R --show
    emu_options setters=0 output=1 load=10 uset=1200 iset=2000
    for key in voltage voltage_target current current_limit enabled regulation ovp_active \
        ovp_threshold ocp_active ocp_threshold; do
        emu_case "get DPS $key" -- -d $D --get $key
    done
    emu_case "get DPS range" --stderr --expect-fail -- -d $D --get range
    emu_options setters=0 id=5015 uset=1200 iset=1234 ovp=4000 ocp=1600
    emu_case "get DPS5015 current_limit" -- -d $D --get current_limit
    emu_case "get DPS5015 ocp_threshold" -- -d $D --get ocp_threshold

    # A setting goes out as asked, alone, in its register.
    emu_options id=60125
    emu_case "set RD6012P voltage_target" --driver rdtech-rd --no-output -- -d $R -c voltage_target=5 --set
    emu_case "set RD6012P voltage_target max" --driver rdtech-rd --no-output -- -d $R -c voltage_target=60 --set
    emu_case "set RD6012P voltage_target 0" --driver rdtech-rd --no-output -- -d $R -c voltage_target=0 --set
    emu_case "set RD6012P voltage_target exact" --driver rdtech-rd --no-output -- -d $R -c voltage_target=12.5 --set
    emu_case "set RD6012P current_limit" --driver rdtech-rd --no-output -- -d $R -c current_limit=1 --set
    emu_case "set RD6012P current_limit max" --driver rdtech-rd --no-output -- -d $R -c current_limit=6 --set
    emu_case "set RD6012P enabled on" --driver rdtech-rd --no-output -- -d $R -c enabled=on --set
    emu_case "set RD6012P enabled off" --driver rdtech-rd --no-output -- -d $R -c enabled=off --set
    emu_case "set RD6012P ovp_threshold" --driver rdtech-rd --no-output -- -d $R -c ovp_threshold=62.5 --set
    emu_case "set RD6012P ocp_threshold" --driver rdtech-rd --no-output -- -d $R -c ocp_threshold=6.5 --set
    emu_case "set RD6012P range" --driver rdtech-rd --no-output -- -d $R -c range=12A --set
    emu_case "set RD6012P range back" --driver rdtech-rd --no-output -- -d $R -c range=6A --set
    emu_case "set RD6012P no such range" --driver rdtech-rd --stderr --expect-fail -- -d $R -c range=1A --set
    emu_case "set then read back" --driver rdtech-rd -- -d $R -c voltage_target=12.5 --get voltage_target
    emu_case "set the range then read back" --driver rdtech-rd -- -d $R -c range=12A --get range
    emu_options id=60125 iset=1500
    emu_case "set the range changes the scale" --driver rdtech-rd -- -d $R -c range=12A --get current_limit
    emu_case "set the range changes the list" --driver rdtech-rd --keep-header -- -d $R -c range=12A --show
    emu_options id=60125
    # Not on a boundary, where the two tools part ways.
    emu_case "set RD6012P voltage_target 3.3" --driver rdtech-rd --no-output -- -d $R -c voltage_target=3.3 --set
    emu_case "set RD6012P current_limit 0.07" --driver rdtech-rd --no-output -- -d $R -c current_limit=0.07 --set
    # Values no register holds.
    emu_case "set voltage_target over the list" --driver rdtech-rd --no-output -- -d $R -c voltage_target=66 --set
    emu_case "set voltage_target negative" --driver rdtech-rd --no-output -- -d $R -c voltage_target=-1 --set
    emu_case "set voltage_target nan" --driver rdtech-rd --no-output -- -d $R -c voltage_target=nan --set
    emu_case "set current_limit over the list" --driver rdtech-rd --no-output -- -d $R -c current_limit=6.5 --set
    emu_case "set ovp_threshold too wide" --driver rdtech-rd --no-output -- -d $R -c ovp_threshold=70 --set
    emu_case "set ocp_threshold negative" --driver rdtech-rd --no-output -- -d $R -c ocp_threshold=-1 --set
    emu_case "set voltage_target -0" --driver rdtech-rd --no-output -- -d $R -c voltage_target=-0 --set
    # The order on the wire: the setpoints, then the output. Hash table order
    # under sigrok-cli, so in these two it is the output first.
    emu_case "set the output after a voltage" --driver rdtech-rd --no-output -- -d $R -c voltage_target=5:enabled=on --set
    emu_case "set the output before a voltage" --driver rdtech-rd --no-output -- -d $R -c enabled=on:voltage_target=5 --set
    emu_case "set all the setpoints" --driver rdtech-rd --no-output -- -d $R -c enabled=on:current_limit=1:voltage_target=5:ovp_threshold=6:ocp_threshold=2 --set
    emu_case "set the output off first" --driver rdtech-rd --no-output -- -d $R -c voltage_target=5:enabled=off --set
    emu_case "set each -c, output first" --driver rdtech-rd --no-output -- -d $R -c enabled=on -c voltage_target=5 --set
    # Nothing is sent unless all of -c is good, and above all no output.
    emu_case "output with a voltage too high" --driver rdtech-rd --no-output -- -d $R -c enabled=on:voltage_target=99 --set
    emu_case "output with an unknown option" --driver rdtech-rd --expect-fail --stderr -- -d $R -c enabled=on:volage_target=5 --set
    emu_case "output with a bad range" --driver rdtech-rd --expect-fail --stderr -- -d $R -c enabled=on:range=1A --set
    emu_case "output with a voltage that is no number" --driver rdtech-rd --no-output -- -d $R -c enabled=on:voltage_target=abc --set
    # One range: DPS, where the lock is a write as well. An open locks the
    # panel, a close unlocks it, and the settings are zero under libsigrok.
    emu_options
    emu_case "set DPS voltage_target" --no-output -- -d $D -c voltage_target=5 --set
    emu_case "set DPS current_limit" --no-output -- -d $D -c current_limit=1.25 --set
    emu_case "set DPS enabled on" --no-output -- -d $D -c enabled=on --set
    emu_case "set DPS enabled off" --no-output -- -d $D -c enabled=off --set
    emu_case "set DPS ovp_threshold" --no-output -- -d $D -c ovp_threshold=45 --set
    emu_case "set DPS ocp_threshold" --no-output -- -d $D -c ocp_threshold=5.5 --set
    emu_case "set DPS the output after a voltage" --no-output -- -d $D -c voltage_target=5:enabled=on --set
    emu_case "set DPS range" --expect-fail --stderr -- -d $D -c range=5A --set
    emu_case "set DPS then read back" -- -d $D -c voltage_target=12.5 --get voltage_target
    emu_options id=60061
    emu_case "set RD6006 voltage_target" --driver rdtech-rd --no-output -- -d $R -c voltage_target=5 --set
    # Refused writes.
    emu_options id=60125 faults=3:exc04
    emu_case "set refused" --driver rdtech-rd --stderr --expect-fail -- -d $R -c voltage_target=5 --set
    emu_options id=60125 faults=3:exc06
    emu_case "set the output refused" --driver rdtech-rd --stderr --expect-fail -- -d $R -c enabled=on --set
    emu_options id=60125 faults=3:silent
    emu_case "set no answer" --driver rdtech-rd --stderr --expect-fail -- -d $R -c voltage_target=5 --set
    emu_options id=60125 faults=3:count
    emu_case "set wrong echo" --driver rdtech-rd --stderr --expect-fail -- -d $R -c voltage_target=5 --set
    emu_options id=60125 faults=3:crc
    emu_case "set bad crc" --driver rdtech-rd --stderr --expect-fail -- -d $R -c voltage_target=5 --set
    # The lock's own failures: an open fails, a close does not care.
    emu_options setters=0 faults=2:exc04
    emu_case "open: the lock refused" --stderr --expect-fail -- -d $D --samples 1
    emu_case "show: the lock refused" --stderr --expect-fail --keep-header -- -d $D --show
    emu_options setters=0 faults=7:exc04
    emu_case "close: the unlock refused" -- -d $D --samples 1
    # A refused read in --get.
    emu_options setters=0 id=60125 faults=3:exc04,4:exc04,5:exc04
    emu_case "get refused" --driver rdtech-rd --stderr --expect-fail -- -d $R --get voltage_target
    emu_options setters=0 id=60125 faults=3:silent,4:silent,5:silent
    emu_case "get no answer" --driver rdtech-rd --stderr --expect-fail -- -d $R --get voltage_target
    emu_options setters=0 id=60125 faults=4:silent,5:silent,6:silent
    emu_case "get second block no answer" --driver rdtech-rd --stderr --expect-fail -- -d $R --get voltage_target
    emu_options setters=0 id=60125 faults=3:silent
    emu_case "get one no answer, then fine" --driver rdtech-rd -- -d $R --get voltage_target

    # A capture: both blocks, per sample.
    emu_options setters=0 id=60125 output=1 load=10 uset=12000 iset=20000
    emu_case "samples 1" --driver rdtech-rd -- -d $R --samples 1
    emu_case "samples 3" --driver rdtech-rd -- -d $R --samples 3
    emu_case "samples csv" --driver rdtech-rd -- -d $R --samples 2 -O csv:header=false:time=false
    emu_case "channel V only" --driver rdtech-rd -- -d $R -C V --samples 2
    emu_case "channels I and P" --driver rdtech-rd -- -d $R -C I,P --samples 2
    emu_case "channel V only csv" --driver rdtech-rd -- -d $R -C V --samples 2 -O csv:header=false:time=false
    emu_case "config limit_samples 0" --driver rdtech-rd --stderr --expect-fail -- -d $R -c limit_samples=0 --samples 1
    # A time limit: how many polls that is depends on the clock, so both are
    # kept up to the first poll of the capture.
    emu_case "time" --driver rdtech-rd --filter '4,$d' \
        --tx-filter '/^bulk_in 82 9 -> /{x;s/^/x/;/^xxx/{x;q};x}' -- -d $R --time 500ms
    emu_options setters=0 id=60125 uout=12000,12345,0,60000 iout=1000,1,0,60000 power=1200,1,0,65535
    emu_case "readings in turn" --driver rdtech-rd -- -d $R --samples 4
    # What changes under a capture comes as META after that frame, the range
    # also changing the next frame's scale.
    emu_options setters=0 id=60125 protect=0,1,0,2,0 cc=0,0,1,1,0 enable=1,1,0,1,1
    emu_case "state changes" --driver rdtech-rd --keep-header -- -d $R --samples 5 -O analog
    emu_case "state changes default output" --driver rdtech-rd --keep-header -- -d $R --samples 5
    emu_options setters=0 id=60125 range=0,0,1,1,0,0 iset=10000 iout=10000
    emu_case "range changes" --driver rdtech-rd --keep-header -- -d $R --samples 5 -O analog
    # The output goes on after all else: the setpoints, the output, the reads.
    emu_options id=60125 load=10
    emu_case "capture with the output on" --driver rdtech-rd --keep-header -- -d $R -c enabled=on --samples 2 -O analog
    emu_case "capture with setpoints and the output" --driver rdtech-rd --keep-header -- -d $R -c voltage_target=12:current_limit=2:enabled=on --samples 2 -O analog
    emu_case "capture with the output before the setpoints" --driver rdtech-rd --keep-header -- -d $R -c enabled=on -c voltage_target=12 --samples 2 -O analog
    emu_case "capture with a limit that is no number" --driver rdtech-rd --expect-fail --stderr -- -d $R -c enabled=on --samples abc
    # One range.
    emu_options setters=0 id=5005 output=1 load=10 uset=1200 iset=2000
    emu_case "DPS samples" -- -d $D --samples 2
    emu_case "DPS csv" -- -d $D --samples 2 -O csv:header=false:time=false
    emu_options id=5005 load=10
    emu_case "DPS capture with setpoints" -- -d $D -c voltage_target=12:enabled=on --samples 2
    emu_options setters=0 id=60061 output=1 load=10 uset=1200 iset=2000
    emu_case "RD6006 samples" --driver rdtech-rd -- -d $R --samples 2

    # Answers gone wrong in a capture. A failed start, and a read retried.
    emu_options setters=0 id=60125 faults=3:exc04,4:exc04,5:exc04
    emu_case "start refused" --driver rdtech-rd --stderr --expect-fail --filter 's/(event source) 0x[0-9a-f]+/\1 P/' -- -d $R --samples 1
    emu_options setters=0 id=60125 faults=6:crc,7:crc
    emu_case "a poll retried" --driver rdtech-rd -- -d $R --samples 2
    # A poll that fails three times: the C returns its negative result to the
    # session as a truth, takes no sample and asks again.
    emu_options setters=0 id=60125 faults=5:silent,6:silent,7:silent
    emu_case "a poll lost" --driver rdtech-rd -- -d $R --samples 1
    emu_options setters=0 id=60125 answers=4
    emu_case "silent in a capture" --hangs --timeout 8 --no-output -- -d $R --samples 2
    emu_options setters=0 id=60125 answers=6
    emu_case "silent after a sample" --hangs --timeout 8 -- -d $R --samples 3

    # The USB side misbehaving, the shell's options.
    emu_options setters=0 id=60125 chunk=1
    emu_case "one byte per transfer" --driver rdtech-rd -- -d $R --samples 2
    # No in-stall=: whether the stall takes a reply or a read with nothing
    # to come depends on the kernel's queued reads, and three tries is all a
    # read has here.
    emu_options setters=0 stall=line
    emu_case "line coding stalled" --keep-header -- -d $D --scan
    emu_options setters=0 stall=lines
    emu_case "control lines stalled" --keep-header -- -d $D --scan
    emu_options setters=0 id=60125 answers=6
    emu_case "unplug in a capture" --unplug model --hangs --timeout 8 -- -d $R --samples 3
    emu_case "interrupt" --expect-fail --timeout 8 --interrupt model -- -d $R --continuous
    emu_options setters=0 answers=0
    emu_case "unplug in the scan" --unplug model --timeout 30 --keep-header --no-output -- -d $D --scan
    emu_options
}

# See emu_options in tools/diffcli.sh.
emu_conns() {
    D=rdtech-dps:conn=$EMU_TTY
    R=rdtech-rd:conn=$EMU_TTY
}
