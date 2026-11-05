# ICStation relay boards (emu/src/models/icstation-usbrelay.rs) on the CDC-ACM
# shell. Sourced by tools/diffcli.sh --emu; see docs/USB.md, "Writing a serial
# model".
EMU_MODEL=icstation-usbrelay
EMU_CDC=1
EMU_DRIVER=icstation-usbrelay
EMU_FULL=$EMU_DRIVER
EMU_GCOV=icstation-usbrelay
EMU_GCOV_SHARED="src/serial.c src/serial_libsp.c"

emu_cases() {
    emu_conns
    # oxrok's log prefix, on its side only.
    F='s/^oxrok: err: oxrok::drivers::icstation_usbrelay: /sr: icstation-usbrelay: /
s/^oxrok: warn: oxrok::drivers::icstation_usbrelay: /sr: icstation-usbrelay: /'
    CONN_BUG="config_get's first switch breaks out for SR_CONF_CONN (api.c:138-140) into the second, whose default is SR_ERR_NA (api.c:153-154), so the conn that devopts advertises (api.c:36) is never answered; oxrok answers it"
    BOOL_BUG="sr_parse_boolstring() takes 1x as on (strutil.c:1315-1319), oxrok refuses it"
    LIMIT_BUG="sigrok-cli switches the relays, then fails on the limit (session.c:865); oxrok refuses first"
    # Each line of api.c's table.
    for b in ICSE012A ICSE013A ICSE014A; do
        emu_options board=$b
        emu_case "scan $b" --keep-header -- -d $I --scan
    done
    emu_options
    emu_case "show" --oracle-differs "$CONN_BUG" --keep-header -- -d $I --show
    emu_case "show R1" --keep-header -- -d $I -g R1 --show
    emu_case "show no such group" --keep-header --stderr --expect-fail -- -d $I -g R5 --show
    emu_case "no conn" --keep-header --no-output -- -d icstation-usbrelay --scan
    emu_case "no conn stderr" --keep-header --stderr --expect-fail -- -d icstation-usbrelay --show
    emu_case "no such node" --keep-header --no-output -- -d icstation-usbrelay:conn=/dev/ttyACM97 --scan
    emu_case "serialcomm as it is" --keep-header -- -d $I:serialcomm=9600/8n1 --scan
    emu_case "serialcomm misset" --stderr --filter "$F" --keep-header -- -d $I:serialcomm=19200/8n1 --scan
    # libsigrok raises the lines at its open before it refuses the setting, and
    # oxrok parses the setting first.
    emu_case "serialcomm no such" --tx-filter '/^ctl_out/d' --keep-header --no-output -- -d $I:serialcomm=fast --scan
    emu_options uart=19200/8n1
    emu_case "serialcomm on a changed line" --keep-header -- -d $I:serialcomm=19200/8n1 --scan

    # What a get reads: the cache, with nothing on the wire after the start.
    emu_options
    emu_case "get R1" --keep-header -- -d $I -g R1 --get enabled
    emu_case "get conn" --oracle-bug "$CONN_BUG" --keep-header -- -d $I --get conn
    emu_case "get enabled no group" --keep-header --stderr --expect-fail -- -d $I --get enabled
    emu_case "get after set" --keep-header -- -d $I -g R2 -c enabled=on --get enabled

    # Settings: one byte, active low, all relays in it.
    emu_case "set R1 on" --keep-header --no-output -- -d $I -g R1 --set -c enabled=on
    emu_case "set R4 on" --keep-header --no-output -- -d $I -g R4 --set -c enabled=on
    emu_case "set R1 off" --keep-header --no-output -- -d $I -g R1 --set -c enabled=off
    emu_case "set all on" --keep-header --no-output -- -d $I --set -c enabled=on
    emu_case "set all off" --keep-header --no-output -- -d $I --set -c enabled=off
    emu_case "set two relays" --keep-header --no-output -- -d $I --set -c channel_group=R1:enabled=on -c channel_group=R3:enabled=on
    emu_case "set all but one" --keep-header --no-output -- -d $I --set -c enabled=on -c channel_group=R2:enabled=off
    emu_case "set malformed" --oracle-differs "$BOOL_BUG" --keep-header --expect-fail --stderr -- -d $I -g R1 --set -c enabled=1x
    emu_options board=ICSE014A
    emu_case "set R8 on" --keep-header --no-output -- -d $I -g R8 --set -c enabled=on
    emu_case "set all on 8" --keep-header --no-output -- -d $I --set -c enabled=on
    emu_options board=ICSE013A
    emu_case "set all on 2" --keep-header --no-output -- -d $I --set -c enabled=on
    emu_case "set R3 on 2" --keep-header --stderr --expect-fail -- -d $I -g R3 --set -c enabled=on

    # The driver's dummy start.
    emu_options
    emu_case "capture samples" --keep-header --stderr --expect-fail -- -d $I --samples 1
    emu_case "capture enabled" --oracle-bug "$LIMIT_BUG" --keep-header --expect-fail --stderr -- -d $I -c enabled=on --samples 1
    emu_case "capture continuous" --keep-header --stderr --expect-fail -- -d $I --continuous
    emu_case "capture time" --keep-header --stderr --expect-fail -- -d $I --time 10
    # With no limit to set, the session refuses: no channels (session.c:790-793).
    emu_case "lib capture" --libsigrok --keep-header --stderr --expect-fail -- -d $I

    # Scans that find nothing. The request on a started board is a relay byte,
    # which no golden can show; the model's unit tests hold that.
    emu_options started=1
    emu_case "scan a started board" --stderr --filter "$F" --keep-header -- -d $I --scan
    emu_options silent=1
    emu_case "scan silent" --stderr --filter "$F" --keep-header -- -d $I --scan
    emu_options answer=12
    emu_case "scan unknown id" --stderr --filter "$F" --keep-header -- -d $I --scan
    emu_options answer=abcd
    emu_case "scan two bytes" --keep-header --no-output -- -d $I -g R1 --set -c enabled=on
    emu_options late=2
    emu_case "scan late" --keep-header -- -d $I --scan
    # A read that fails: the unplug comes while the tool waits for the id.
    emu_options silent=1
    emu_case "unplug waiting for the id" --unplug out=2:1 --stderr --filter "$F" --keep-header -- -d $I --scan
    emu_options
}

emu_conns() {
    I=icstation-usbrelay:conn=$EMU_TTY
}
