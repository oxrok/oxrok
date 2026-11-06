# The HP 59306A relay actuator (emu/src/models/hp-59306a.rs), which really sits on
# HP-IB, on a socket alone: oxrok has no GPIB, and tcp-raw takes the same commands
# from the same scpi.c. Sourced by tools/diffcli.sh --emu; see docs/USB.md, "Writing
# a SCPI model". It sends nothing back, so the .tx is all most cases hold.
EMU_MODEL=hp-59306a
EMU_TCP=only
EMU_DRIVER=hp-59306a
EMU_GCOV=hp-59306a
EMU_GCOV_SHARED="src/scpi/scpi.c src/scpi/scpi_tcp.c src/tcp.c"

emu_cases() {
    D=hp-59306a
    T=$D:conn=tcp-raw/127.0.0.1/$EMU_TCP_PORT

    # What a scan is: a connection and nothing on it. Whatever opens is a 59306A.
    emu_options
    emu_case "scan" --keep-header -- -d $T --scan
    emu_case "scan tcp-rigol" --keep-header -- -d $D:conn=tcp-rigol/127.0.0.1/$EMU_TCP_PORT --scan
    emu_case "scan with no conn" --keep-header --no-output -- -d $D --scan
    emu_case "scan with nobody there" --keep-header --no-output -- -d $D:conn=tcp-raw/127.0.0.1/1 --scan
    emu_case "scan with no port" --keep-header --no-output -- -d $D:conn=tcp-raw/127.0.0.1 --scan
    emu_case "scan of an unlisted option" --keep-header --no-output -- -d $T:serialcomm=9600/8n1 --scan
    emu_case "show with no conn" --stderr --expect-fail -- -d $D --show

    emu_case "show" --keep-header -- -d $T --show
    emu_case "show R1" --keep-header -- -d $T -g R1 --show
    emu_case "show R6" --keep-header -- -d $T -g R6 --show
    emu_case "show R7" --stderr --expect-fail -- -d $T -g R7 --show
    emu_case "get conn" -- -d $T --get conn
    emu_case "get conn of a group" --stderr --expect-fail -- -d $T -g R1 --get conn
    emu_case "get enabled" --stderr --expect-fail -- -d $T --get enabled
    emu_case "get enabled of a group" --stderr --expect-fail -- -d $T -g R1 --get enabled

    # The commands. A relay's number comes from its group, on and off.
    emu_case "all on" --no-output -- -d $T --config enabled=on --set
    emu_case "all off" --no-output -- -d $T --config enabled=off --set
    emu_case "R1 on" --no-output -- -d $T -g R1 --config enabled=on --set
    emu_case "R1 off" --no-output -- -d $T -g R1 --config enabled=off --set
    emu_case "R2 on" --no-output -- -d $T -g R2 --config enabled=on --set
    emu_case "R3 off" --no-output -- -d $T -g R3 --config enabled=off --set
    emu_case "R4 on" --no-output -- -d $T -g R4 --config enabled=on --set
    emu_case "R5 off" --no-output -- -d $T -g R5 --config enabled=off --set
    emu_case "R6 on" --no-output -- -d $T -g R6 --config enabled=on --set
    emu_case "R6 off" --no-output -- -d $T -g R6 --config enabled=off --set
    emu_case "group in the config" --no-output -- -d $T --config channel_group=R2:enabled=off --set
    emu_case "two groups in two configs" --no-output -- -d $T --config channel_group=R1:enabled=off --config channel_group=R2:enabled=off --set
    emu_case "all but one" --no-output -- -d $T --config enabled=on --config channel_group=R1:enabled=off --set
    emu_case "one config keeps the last" --no-output -- -d $T --config enabled=on:enabled=off --set

    # The forms a value takes.
    for form in true yes 1 false no 0; do
        emu_case "enabled=$form" --no-output -- -d $T --config enabled=$form --set
    done
    emu_case "enabled empty" --no-output -- -d $T --config enabled= --set
    emu_case "enabled=maybe" --no-output -- -d $T --config enabled=maybe --set

    # What it refuses, with nothing sent.
    emu_case "unknown key" --stderr --expect-fail -- -d $T --config samplerate=1k --set
    emu_case "conn cannot be set" --stderr --expect-fail -- -d $T --config conn=x --set
    emu_case "set in an unknown group" --stderr --expect-fail -- -d $T -g R7 --config enabled=on --set
    emu_case "set with nobody there" --stderr --expect-fail -- -d $D:conn=tcp-raw/127.0.0.1/1 --config enabled=on --set

    # It acquires nothing.
    emu_case "samples" --keep-header --stderr --expect-fail -- -d $T --samples 1
    emu_case "time" --keep-header --stderr --expect-fail -- -d $T --time 100
    emu_case "continuous" --keep-header --stderr --expect-fail -- -d $T --continuous
    emu_case "lib capture has no enabled channels" --libsigrok --expect-fail --stderr -- -d $T --continuous

    # A bridge that talks: nothing reads it.
    emu_options banner=hello
    emu_case "set with a bridge talking" --no-output -- -d $T --config enabled=on --set

    # The scan's connection closes on it, and the next is accepted and closed, as a
    # bridge with one client does: the write goes into nothing.
    emu_options mark-conn=1
    emu_case "set on a line that closes" --no-output --unplug tcp-model -- -d $T --config enabled=on --set
}
