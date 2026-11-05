# Maynuo M97 and M98 electronic loads (emu/src/models/maynuo-m97.rs), a Modbus
# RTU slave behind a UART, on the CDC-ACM shell. Sourced by tools/diffcli.sh
# --emu; see docs/USB.md, "Writing a serial model".
EMU_MODEL=maynuo-m97
EMU_CDC=1
EMU_DRIVER=maynuo-m97
EMU_GCOV=maynuo-m97
EMU_GCOV_SHARED="src/serial.c src/serial_libsp.c src/modbus/modbus.c src/modbus/modbus_serial_rtu.c src/crc.c"

emu_cases() {
    emu_conns

    # Each line of api.c's supported_models[], and how a version shows.
    for id in 28 101; do
        emu_options setters=0 id=$id
        emu_case "scan model $id" --keep-header -- -d $M --scan
    done
    emu_options setters=0 version=5
    emu_case "scan version under ten" --keep-header -- -d $M --scan
    emu_options setters=0 version=65535
    emu_case "scan version corner" --keep-header -- -d $M --scan
    # One of the lines api.c has commented out, and what no line has.
    emu_options setters=0 id=53
    emu_case "scan model commented out" --stderr --keep-header -- -d $M --scan
    emu_options setters=0 id=9999
    emu_case "scan unknown model" --stderr --keep-header -- -d $M --scan
    emu_options setters=0
    emu_case "no conn" --keep-header --no-output -- -d maynuo-m97 --scan
    emu_case "no such node" --keep-header --no-output -- -d maynuo-m97:conn=/dev/ttyACM97 --scan

    # The line and the address.
    emu_case "serialcomm misset" --keep-header --no-output -- -d $M:serialcomm=19200/8n1 --scan
    emu_case "serialcomm as it is" --keep-header -- -d $M:serialcomm=9600/8n1 --scan
    emu_case "serialcomm no such" --keep-header --no-output -- -d $M:serialcomm=fast --scan
    emu_options setters=0 uart=19200/8n1
    emu_case "serialcomm on a changed line" --keep-header -- -d $M:serialcomm=19200/8n1 --scan
    emu_options setters=0 addr=5
    emu_case "modbusaddr of the slave" --keep-header -- -d $M:modbusaddr=5 --scan
    emu_case "modbusaddr default misses it" --keep-header --no-output -- -d $M --scan
    emu_options setters=0
    emu_case "modbusaddr other" --keep-header --no-output -- -d $M:modbusaddr=7 --scan
    emu_case "modbusaddr 0 is a broadcast" --keep-header --no-output -- -d $M:modbusaddr=0 --scan
    emu_case "modbusaddr over 8 bits" --keep-header -- -d $M:modbusaddr=257 --scan

    # What a slave can answer in the probe, with no second try.
    for code in 01 02 03 04 05 06 08 0a 0b 42; do
        emu_options setters=0 faults=1:exc$code
        emu_case "scan exception $code" --stderr --keep-header -- -d $M --scan
    done
    for kind in silent addr count func first nocrc; do
        emu_options setters=0 faults=1:$kind
        emu_case "scan $kind" --keep-header --no-output -- -d $M --scan
    done
    emu_options setters=0 faults=1:crc
    emu_case "scan crc" --stderr --keep-header -- -d $M --scan
    emu_options setters=0 faults=1:short
    emu_case "scan short" --stderr --keep-header -- -d $M --scan
    emu_options setters=0 faults=1:extra
    emu_case "scan two bytes too many" --keep-header -- -d $M --scan

    # --show: what a group lists, and what the device does.
    emu_options setters=0
    emu_case "show" --keep-header -- -d $M --show
    emu_case "show group" --keep-header -- -d $M -g 1 --show
    emu_case "show no such group" --stderr --expect-fail --keep-header -- -d $M -g 2 --show
    emu_options setters=0 id=101 ifix=2.5 ufix=24 imax=20 umax=100 source=24 coils=0x510,0x520
    emu_case "show M9812 on" --keep-header -- -d $M -g 1 --show

    # --get, in the group each key is listed for. What is outside it is not
    # reached: sigrok-cli's check asks the list of what was named.
    emu_options setters=0 ifix=2.5 ufix=24 imax=20 umax=100 source=24.5 coils=0x510,0x520,0x521,0x523
    for key in voltage voltage_target current current_limit enabled regulation \
        ovp_enabled ovp_active ovp_threshold ocp_enabled ocp_active ocp_threshold otp otp_active; do
        emu_case "get $key" -- -d $M -g 1 --get $key
    done
    emu_options setters=0
    for key in enabled ovp_active ocp_active otp_active; do
        emu_case "get $key off" -- -d $M -g 1 --get $key
    done
    emu_case "get limit_samples" -- -d $M --get limit_samples
    emu_case "get limit_time" -- -d $M --get limit_time
    emu_case "get limit_samples in the group" --stderr --expect-fail -- -d $M -g 1 --get limit_samples
    emu_case "get voltage out of the group" --stderr --expect-fail -- -d $M --get voltage
    emu_case "get conn" --stderr --expect-fail -- -d $M --get conn
    emu_case "get power unlisted" --stderr --expect-fail -- -d $M -g 1 --get power
    # The byte of a one coil read holds more than the coil, as in the manual's
    # example: 0x48 reads as off, and what the next coil says stays there.
    emu_options setters=0 coils=0x513,0x516
    emu_case "get enabled off in the manual's byte" -- -d $M -g 1 --get enabled
    emu_options setters=0 coils=0x521,0x522
    emu_case "get ocp_active off beside others" -- -d $M -g 1 --get ocp_active
    emu_options setters=0 coils=0x524
    emu_case "get otp_active off beside reverse" -- -d $M -g 1 --get otp_active
    # A regulation is UNREG, then the mode: every name protocol.c has, and what
    # it does not name.
    for mode in 1 2 3 4 20 25 26 27 30 31 32 33 34 36 38 39 41 99; do
        emu_options setters=0 setmode=$mode
        emu_case "get regulation mode $mode" -- -d $M -g 1 --get regulation
    done
    emu_options setters=0 setmode=513
    emu_case "get regulation only the low byte counts" -- -d $M -g 1 --get regulation
    emu_options setters=0 setmode=256
    emu_case "get regulation the mode in the high byte" -- -d $M -g 1 --get regulation
    emu_options setters=0 coils=0x525 setmode=2
    emu_case "get regulation unregulated" -- -d $M -g 1 --get regulation

    # A get that fails: what is left of the device, each key's read refused.
    for key in voltage voltage_target current current_limit enabled regulation ovp_active \
        ovp_threshold ocp_active ocp_threshold otp_active; do
        emu_options setters=0 faults=3:exc04
        emu_case "get refused $key" --stderr --expect-fail -- -d $M -g 1 --get $key
    done
    emu_options setters=0 faults=4:exc04
    emu_case "get regulation, the mode refused" --stderr --expect-fail -- -d $M -g 1 --get regulation
    for kind in silent crc addr short count func first nocrc; do
        emu_options setters=0 faults=3:$kind
        emu_case "get voltage $kind" --stderr --expect-fail -- -d $M -g 1 --get voltage
        emu_case "get enabled $kind" --stderr --expect-fail -- -d $M -g 1 --get enabled
    done

    # A setting goes out as asked, in its register.
    emu_options
    for set in voltage_target=5 voltage_target=150 voltage_target=0 voltage_target=3.3 \
        voltage_target=0.001 current_limit=1 current_limit=30 current_limit=0.0001 \
        current_limit=1.2345 ovp_threshold=62.5 ocp_threshold=6.5; do
        emu_case "set $set" --no-output -- -d $M -g 1 -c $set --set
    done
    emu_case "set enabled on" --no-output -- -d $M -g 1 -c enabled=on --set
    emu_case "set enabled off" --no-output -- -d $M -g 1 -c enabled=off --set
    emu_options setmode=2
    emu_case "set enabled keeps the mode CV" --no-output -- -d $M -g 1 -c enabled=on --set
    emu_options setmode=256
    emu_case "set enabled the mode in the high byte" --no-output -- -d $M -g 1 -c enabled=on --set
    emu_options
    emu_case "set then read back" -- -d $M -g 1 -c voltage_target=12.5 --get voltage_target
    emu_case "set a float then read back" -- -d $M -g 1 -c current_limit=3.3 --get current_limit
    emu_case "set enabled then read back" -- -d $M -g 1 -c enabled=on --get enabled
    # What the list does not hold. A float goes out whatever it is.
    emu_case "set voltage_target over the list" --no-output -- -d $M -g 1 -c voltage_target=151 --set
    emu_case "set voltage_target negative" --no-output -- -d $M -g 1 -c voltage_target=-1 --set
    emu_case "set current_limit over the list" --no-output -- -d $M -g 1 -c current_limit=31 --set
    emu_case "set voltage_target nan" --no-output -- -d $M -g 1 -c voltage_target=nan --set
    emu_case "set ocp_threshold inf" --no-output -- -d $M -g 1 -c ocp_threshold=inf --set
    emu_case "set ovp_threshold negative" --no-output -- -d $M -g 1 -c ovp_threshold=-1 --set
    emu_case "set voltage_target no number" --no-output -- -d $M -g 1 -c voltage_target=abc --set
    # What cannot be set, or not there.
    emu_case "set voltage read only" --stderr --expect-fail -- -d $M -g 1 -c voltage=5 --set
    emu_case "set voltage_target out of the group" --stderr --expect-fail -- -d $M -c voltage_target=5 --set
    emu_case "set limit_samples in the group" --stderr --expect-fail -- -d $M -g 1 -c limit_samples=5 --set
    emu_case "set limit_samples" --no-output -- -d $M -c limit_samples=5 --set
    emu_case "set no such key" --stderr --expect-fail -- -d $M -g 1 -c volage_target=5 --set
    # The order on the wire: the setpoint and the input.
    emu_case "set the input after a setpoint" --no-output -- -d $M -g 1 -c current_limit=1:enabled=on --set
    emu_case "set the input before a setpoint" --no-output -- -d $M -g 1 -c enabled=on:current_limit=1 --set
    emu_case "set all the setpoints" --no-output -- -d $M -g 1 -c enabled=on:current_limit=1:voltage_target=5:ovp_threshold=6:ocp_threshold=2 --set
    emu_case "set each -c, input first" --no-output -- -d $M -g 1 -c enabled=on -c current_limit=1 --set
    emu_case "set the input off first" --no-output -- -d $M -g 1 -c current_limit=1:enabled=off --set
    emu_case "the input with an unknown option" --stderr --expect-fail -- -d $M -g 1 -c enabled=on:volage_target=5 --set
    emu_case "the input with a value that is no number" --no-output -- -d $M -g 1 -c enabled=on:current_limit=abc --set

    # Writes refused. Each of the input's three steps.
    for set in voltage_target=5 current_limit=1 ovp_threshold=45 ocp_threshold=5; do
        emu_options faults=3:exc04
        emu_case "set refused ${set%%=*}" --stderr --expect-fail -- -d $M -g 1 -c $set --set
    done
    for step in 3 4 5; do
        emu_options faults=$step:exc04
        emu_case "set enabled refused at step $step" --stderr --expect-fail -- -d $M -g 1 -c enabled=on --set
    done
    for kind in silent crc count short; do
        emu_options faults=3:$kind
        emu_case "set $kind" --stderr --expect-fail -- -d $M -g 1 -c current_limit=1 --set
    done

    # PC1, which nothing looks at the answer to.
    emu_options setters=0 faults=2:exc04
    emu_case "open: PC1 refused" -- -d $M -g 1 --get voltage
    emu_options setters=0 faults=2:silent
    emu_case "open: PC1 no answer" -- -d $M -g 1 --get voltage
    emu_options setters=0 faults=4:exc04
    emu_case "close: PC1 refused" -- -d $M -g 1 --get voltage
    emu_options setters=0 faults=4:silent
    emu_case "close: PC1 no answer" -- -d $M -g 1 --get voltage
    # What a bad reply leaves fails the next, where nothing clears it.
    emu_options setters=0 faults=2:extra
    emu_case "open: PC1 two bytes too many" --stderr --expect-fail -- -d $M -g 1 --get voltage

    # The ways out: the .tx's last write is PC1 off, or is not.
    emu_options setters=0
    emu_case "way out: no number for the samples" --stderr --expect-fail -- -d $M --samples abc
    emu_case "way out: no such channel" --stderr --expect-fail -- -d $M -C Z --samples 1
    emu_case "way out: no such output" --stderr --expect-fail -- -d $M -O nosuch --samples 1
    emu_case "way out: get no such key" --stderr --expect-fail -- -d $M -g 1 --get nosuch

    # A capture sends no request (api.c's start, protocol.c's read of a reply),
    # so it takes no sample: only a time limit ends it.
    emu_options setters=0 source=24 ifix=2 u=24.5 i=2.5
    emu_case "time" --no-output -- -d $M --time 700ms
    emu_case "samples, none ever come" --hangs --timeout 3 --no-output -- -d $M --samples 1
    # sigrok-cli's any key: stdin is at its end, so this stops at once.
    emu_case "continuous" --no-output -- -d $M --continuous
    emu_options setters=0 quiet=10
    emu_case "unplug in a capture" --hangs --timeout 3 --unplug model --no-output -- -d $M --samples 3
    # What it does with a reply that came all the same, a slave no one has:
    # this and no more shows the frames, see unasked= in the model.
    emu_options setters=0 unasked=1
    emu_case "unasked samples 1" -- -d $M --samples 1
    emu_options setters=0 unasked=3 u=12.5,13,0.0001 i=1.25,2,0.0001
    emu_case "unasked samples 3" -- -d $M --samples 3
    emu_case "unasked analog" -- -d $M --samples 3 -O analog
    emu_case "unasked csv" -- -d $M --samples 3 -O csv:header=false:time=false
    emu_case "unasked V1 only" -- -d $M -C V1 --samples 3
    emu_case "unasked I1 only" -- -d $M -C I1 --samples 3
    emu_case "unasked V1 only csv" -- -d $M -C V1 --samples 3 -O csv:header=false:time=false
    emu_case "unasked I1 only csv" -- -d $M -C I1 --samples 3 -O csv:header=false:time=false
    emu_case "unasked limit by config" -- -d $M -c limit_samples=3 --samples 3
    emu_case "config limit_samples 0" --stderr --expect-fail -- -d $M -c limit_samples=0 --samples 1
    emu_options setters=0 unasked=4 u=0,150,1234.5678,-1 i=0,30,0.00012345,-1
    emu_case "unasked corners" -- -d $M --samples 4
    emu_options setters=0 unasked=3 u=inf,-inf,nan i=nan,inf,-inf
    emu_case "unasked no number" -- -d $M --samples 3
    emu_options setters=0 unasked=2 chunk=1
    emu_case "unasked one byte per transfer" -- -d $M --samples 2
    # A setting before a capture: its frames, then nothing. A reply no one asked
    # for would only meet the setting's read, so these have none.
    emu_options ifix=3
    emu_case "capture with the input on" --hangs --timeout 3 --no-output -- -d $M -g 1 -c enabled=on --samples 2
    emu_case "capture with setpoints and the input" --hangs --timeout 3 --no-output -- -d $M -g 1 -c current_limit=2:enabled=on --samples 2
    # More came than the limit: what is left fails PC1 off's reply, and that is
    # not looked at.
    emu_options setters=0 unasked=5
    emu_case "unasked more than the limit" -- -d $M --samples 2
    # A stop from the library, which sigrok-cli has no way to ask for.
    emu_options setters=0 unasked=4
    emu_case "lib unasked stop" --libsigrok -- -d $M --continuous --stop-after 2

    # The USB side misbehaving, the shell's options.
    emu_options setters=0 stall=line
    emu_case "line coding stalled" --keep-header -- -d $M --scan
    emu_options setters=0 stall=lines
    emu_case "control lines stalled" --keep-header -- -d $M --scan
    emu_options setters=0 answers=0
    emu_case "unplug in the scan" --unplug model --timeout 30 --keep-header --no-output -- -d $M --scan
    emu_options
}

# See emu_options in tools/diffcli.sh.
emu_conns() {
    M=maynuo-m97:conn=$EMU_TTY
}
