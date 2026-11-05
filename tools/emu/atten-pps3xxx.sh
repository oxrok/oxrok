# An Atten PPS3203T-3S (emu/src/models/atten-pps3xxx.rs) on the CDC-ACM
# shell. Sourced by tools/diffcli.sh --emu; see docs/USB.md, "Writing a
# serial model".
EMU_MODEL=atten-pps3xxx
EMU_CDC=1
EMU_DRIVER=atten-pps3203
EMU_GCOV=atten-pps3xxx
EMU_GCOV_SHARED="src/serial.c src/serial_libsp.c"

emu_cases() {
    emu_conns
    A1=channel_group=CH1
    # A frame of readings: CH1 12.34 V 0.500 A, CH2 5.00 V 1.250 A, CH3 3.30 V
    # 0.025 A, all on, protection on, series.
    R=aaaa04d201f401f404e2014a001901070100010100000000
    # Scans. A scan asks once and changes nothing: setters=0 flags a setting.
    emu_options setters=0
    emu_case "scan" --keep-header -- -d $A --scan
    emu_case "show" --keep-header -- -d $A --show
    emu_case "show CH1" --keep-header -- -d $A -g CH1 --show
    emu_case "show CH3" --keep-header -- -d $A -g CH3 --show
    emu_case "no such node" --keep-header --no-output -- -d atten-pps3203:conn=/dev/ttyACM97 --scan
    emu_case "no conn" --keep-header --no-output -- -d atten-pps3203 --scan
    emu_case "show without a device" --keep-header --stderr --expect-fail -- -d atten-pps3203 --show
    emu_options setters=0 answers=0
    emu_case "scan no answer" --keep-header --no-output -- -d $A --scan
    emu_options setters=0 replies=aaaa
    emu_case "scan a short answer" --keep-header --no-output -- -d $A --scan
    emu_options setters=0 replies=aa20000000000000000000000000000000000000000000ff
    emu_case "scan another header" --keep-header --no-output -- -d $A --scan
    emu_options setters=0 replies=55aa000000000000000000000000000000000000000000ff
    emu_case "scan no header" --keep-header --no-output -- -d $A --scan
    emu_options setters=0 replies=aaaa00000000000000000000000000000000000000000000ffff
    emu_case "scan a long answer" --keep-header -- -d $A --scan
    emu_options setters=0 chunk=1
    emu_case "scan one byte per transfer" --keep-header -- -d $A --scan

    # The line.
    emu_options setters=0
    emu_case "serialcomm as the default" --keep-header -- -d $A:serialcomm=9600/8n2 --scan
    emu_case "serialcomm one stop bit" --keep-header -- -d $A:serialcomm=9600/8n1 --scan
    emu_case "serialcomm another rate" --keep-header --no-output -- -d $A:serialcomm=19200/8n2 --scan
    emu_case "serialcomm with parity" --keep-header --no-output -- -d $A:serialcomm=9600/8e1 --scan
    emu_case "serialcomm no such thing" --keep-header --no-output -- -d $A:serialcomm=fast --scan
    emu_options setters=0 uart=19200/8n2
    emu_case "serialcomm as the device is set" --keep-header -- -d $A:serialcomm=19200/8n2 --scan
    emu_case "default on a device set otherwise" --keep-header --no-output -- -d $A --scan

    # Before a capture nothing has come from the device for these to say.
    emu_options setters=0
    for key in channel_config ocp_enabled; do
        emu_case "get $key" -- -d $A --get $key
    done
    for g in CH1 CH2 CH3; do
        for key in voltage voltage_target current current_limit enabled; do
            emu_case "get $g $key" -- -d $A -g $g --get $key
        done
    done
    emu_case "get voltage without a group" --stderr --expect-fail -- -d $A --get voltage
    emu_case "get channel_config of a group" --stderr --expect-fail -- -d $A -g CH1 --get channel_config
    emu_case "get in a group that is not" --stderr --expect-fail -- -d $A -g CH4 --get voltage
    emu_options setters=0 replies=$R
    emu_case "get channel_config the device has" -- -d $A --get channel_config
    emu_case "get voltage the device has" -- -d $A -g CH1 --get voltage
    emu_case "get enabled the device has" -- -d $A -g CH1 --get enabled
    emu_case "show the device has" --keep-header -- -d $A --show

    # Settings wait for the close, or the capture, and go out as one frame.
    emu_options
    emu_case "set voltage_target" --no-output -- -d $A -g CH1 -c voltage_target=5 --set
    emu_case "set voltage_target CH2" --no-output -- -d $A -g CH2 -c voltage_target=32 --set
    emu_case "set voltage_target CH3" --no-output -- -d $A -g CH3 -c voltage_target=6 --set
    emu_case "set voltage_target CH3 too high" --stderr --expect-fail -- -d $A -g CH3 -c voltage_target=6.01 --set
    emu_case "set voltage_target too high" --stderr --expect-fail -- -d $A -g CH1 -c voltage_target=32.01 --set
    emu_case "set voltage_target negative" --stderr --expect-fail -- -d $A -g CH1 -c voltage_target=-1 --set
    emu_case "set voltage_target nan" --no-output -- -d $A -g CH1 -c voltage_target=nan --set
    emu_case "set voltage_target -0" --no-output -- -d $A -g CH1 -c voltage_target=-0 --set
    emu_case "set voltage_target 5.1" --no-output -- -d $A -g CH1 -c voltage_target=5.1 --set
    emu_case "set voltage_target 3.3" --no-output -- -d $A -g CH1 -c voltage_target=3.3 --set
    emu_case "set current_limit" --no-output -- -d $A -g CH2 -c current_limit=1.5 --set
    emu_case "set current_limit max" --no-output -- -d $A -g CH3 -c current_limit=3 --set
    emu_case "set current_limit too high" --stderr --expect-fail -- -d $A -g CH3 -c current_limit=3.01 --set
    emu_case "set current_limit nan" --no-output -- -d $A -g CH1 -c current_limit=nan --set
    emu_case "set current_limit 0.29" --no-output -- -d $A -g CH1 -c current_limit=0.29 --set
    emu_case "set enabled on" --no-output -- -d $A -g CH2 -c enabled=on --set
    emu_case "set enabled off" --no-output -- -d $A -g CH2 -c enabled=off --set
    emu_case "set voltage_target without a group" --stderr --expect-fail -- -d $A -c voltage_target=5 --set
    emu_case "set enabled without a group" --stderr --expect-fail -- -d $A -c enabled=on --set
    emu_case "set ocp on" --no-output -- -d $A -c ocp_enabled=on --set
    emu_case "set ocp off" --no-output -- -d $A -c ocp_enabled=off --set
    emu_case "set ocp in a group" --stderr --expect-fail -- -d $A -g CH1 -c ocp_enabled=on --set
    emu_case "set channel_config Series" --no-output -- -d $A -c channel_config=Series --set
    emu_case "set channel_config Parallel" --no-output -- -d $A -c channel_config=Parallel --set
    emu_case "set channel_config Independent" --no-output -- -d $A -c channel_config=Independent --set
    emu_case "set channel_config no such" --stderr --expect-fail -- -d $A -c channel_config=Both --set
    emu_case "set then read back target" -- -d $A -g CH1 -c voltage_target=3.25 --get voltage_target
    emu_case "set then read back enabled" -- -d $A -g CH1 -c enabled=on --get enabled
    emu_case "set then read back mode" -- -d $A -c channel_config=Series --get channel_config
    emu_case "set all of a channel" --no-output -- -d $A -c $A1:voltage_target=5:current_limit=0.5:enabled=on --set
    emu_case "set two channels" --no-output -- -d $A -c $A1:voltage_target=5:enabled=on -c channel_group=CH3:voltage_target=3.25:enabled=on --set
    emu_case "set all at once" --no-output -- -d $A -c ocp_enabled=on:channel_config=Parallel -c $A1:voltage_target=5:current_limit=0.5:enabled=on --set
    emu_case "output with a voltage too high" --stderr --expect-fail -- -d $A -c $A1:enabled=on:voltage_target=99 --set
    emu_case "output with an unknown option" --stderr --expect-fail -- -d $A -c $A1:enabled=on:volage_target=5 --set
    # sigrok-cli sets each -c in turn: the first has been accepted by then.
    emu_case "output then a refused -c" --stderr --expect-fail -- -d $A -c $A1:enabled=on -c voltage_target=5 --set
    emu_case "setpoint then a refused -c" --stderr --expect-fail -- -d $A -c $A1:voltage_target=5 -c channel_group=CH3:voltage_target=7 --set
    # Off when the device says it is on; C thinks it has nothing to do.
    emu_options replies=$R
    emu_case "set off an output that is on" --no-output -- -d $A -g CH1 -c enabled=off --set
    emu_case "set ocp off when on" --no-output -- -d $A -c ocp_enabled=off --set
    emu_case "set Independent when series" --no-output -- -d $A -c channel_config=Independent --set

    # The limits a capture does not have.
    emu_options setters=0
    emu_case "samples refused" --stderr --expect-fail -- -d $A --samples 1
    emu_case "time refused" --stderr --expect-fail -- -d $A --time 100
    emu_case "frames refused" --stderr --expect-fail -- -d $A --frames 1
    emu_case "config limit_samples" --stderr --expect-fail -- -d $A -c limit_samples=1 --continuous

    # sigrok-cli stops a --continuous at the EOF on its stdin: the first
    # answer ends the capture, and no poll follows it.
    emu_options answers=2 replies=$R
    emu_case "one reading" -- -d $A --continuous
    emu_case "one reading analog" -- -d $A --continuous -O analog
    emu_case "one reading csv" -- -d $A --continuous -O csv:header=false:time=false
    emu_case "one reading CH1" -- -d $A -C CH1 --continuous
    emu_case "one reading CH2 CH3" -- -d $A -C CH2,CH3 --continuous
    emu_case "one reading CH3 csv" -- -d $A -C CH3 --continuous -O csv:header=false:time=false
    # What was set goes out at the close.
    emu_case "capture with setpoints" -- -d $A -c $A1:voltage_target=5:current_limit=0.5:enabled=on --continuous
    emu_case "capture with a setting" -- -d $A -c ocp_enabled=on --continuous
    emu_case "capture with a bad setpoint" --stderr --expect-fail -- -d $A -c $A1:enabled=on:voltage_target=99 --continuous

    # The library runs it for longer. A poll of 20 follows each answer, and
    # a stop is noticed at the next answer, of which nothing goes out.
    emu_options
    emu_case "lib two samples" --libsigrok -- -d $A --continuous --stop-after 2
    emu_case "lib four samples" --libsigrok -- -d $A --continuous --stop-after 4
    emu_options output=7
    emu_case "lib turns off what it was not told about" --libsigrok -- -d $A --continuous --stop-after 4
    emu_options
    emu_case "lib stop at the header" --libsigrok --no-output -- -d $A --continuous --stop-after 0
    # A device that stops answering holds it.
    emu_options answers=1
    emu_case "lib silent from the start" --libsigrok --hangs --timeout 2 --no-output -- -d $A --continuous
    emu_options answers=3
    emu_case "lib silent after two" --libsigrok --hangs --timeout 2 --filter '1,6!d' -- -d $A --continuous
    emu_options answers=1
    emu_case "lib stop never ends on silence" --libsigrok --hangs --timeout 2 --no-output -- -d $A --continuous --stop-after 0
    # No framing: a byte too many shifts all that follow.
    emu_options replies=${R}ff,$R
    emu_case "lib a long answer shifts the rest" --libsigrok -- -d $A --continuous --stop-after 4 -O csv:header=false:time=false
    emu_options replies=$R,aaaa04
    emu_case "lib a short answer never ends" --libsigrok --hangs --timeout 2 --no-output -- -d $A --continuous
    emu_options chunk=1
    emu_case "lib one byte per transfer" --libsigrok -- -d $A --continuous --stop-after 2

    # A device that leaves.
    emu_options answers=1
    emu_case "lib unplug in a capture" --libsigrok --hangs --unplug model --timeout 3 --no-output -- -d $A --continuous
    emu_options setters=0 answers=0
    emu_case "unplug in the scan" --unplug model --timeout 30 --keep-header --no-output -- -d $A --scan

    # The USB side misbehaving, the shell's options.
    emu_options setters=0 stall=line
    emu_case "line coding stalled" --keep-header -- -d $A --scan
    emu_options setters=0 stall=lines
    emu_case "control lines stalled" --keep-header -- -d $A --scan
    emu_options setters=0 in-stall=1
    emu_case "a stalled read" --keep-header -- -d $A --scan
    emu_options
}

# See emu_options in tools/diffcli.sh.
emu_conns() {
    A=atten-pps3203:conn=$EMU_TTY
}
