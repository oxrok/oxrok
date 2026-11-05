# Maynuo M97 and M98 electronic loads (emu/src/models/maynuo-m97.rs), a Modbus
# RTU slave behind a UART, on the CDC-ACM shell. Sourced by tools/diffcli.sh
# --emu; see docs/USB.md, "Writing a serial model".
EMU_MODEL=maynuo-m97
EMU_CDC=1
EMU_DRIVER=maynuo-m97
EMU_FULL=$EMU_DRIVER
EMU_GCOV=maynuo-m97
EMU_GCOV_SHARED="src/serial.c src/serial_libsp.c src/modbus/modbus.c src/modbus/modbus_serial_rtu.c src/crc.c"

emu_cases() {
    emu_conns
    # oxrok's log prefix, on its side only, and how its errors read beside the
    # names sr_strerror() has for them.
    F='s/^oxrok: err: oxrok::drivers::maynuo_m97: /sr: maynuo-m97: /
s/^oxrok: err: oxrok::modbus: CRC/sr: modbus_serial: CRC/
s/^oxrok: err: oxrok::modbus: /sr: modbus: /
s/device error: data is invalid/data is invalid/
s/device error: no Modbus reply: [a-zA-Z ]*/generic\/unspecified error/
s/^(Failed to open device): .*$/\1./
s/^(Failed to set device option .[a-z_]+.: invalid argument).*$/\1./'
    # PC1 off, which oxrok sends on every way out and sigrok-cli not on an error
    # (NO_CLOSE_BUG): alone with its echo, or at the end of a line where the frame
    # before it had no answer, so a bulk_out's length goes too (its bytes say it).
    # Such a case holds the C driver to the frames and the words before the close;
    # "way out" below pins the off.
    P='/^bulk_out 02 010505000000cd06 -> 8$/d
/^bulk_in 82 8 -> 010505000000cd06$/d
s/^bulk_out 02 ([0-9a-f]+)010505000000cd06 -> [0-9]+$/bulk_out 02 \1/
s/^bulk_out 02 ([0-9a-f]+) -> [0-9]+$/bulk_out 02 \1/'
    # The polls oxrok sends and sigrok-cli does not (NO_REQUEST_BUG): such a case
    # compares the frames, and "time" and "samples" below pin the polls.
    T='/^bulk_out 02 01030b000004[0-9a-f]{4} -> 8$/d
/^bulk_in 82 13 -> 010308/d'
    # A run of unanswered polls, one .tx line whose length goes by the clock.
    N='s/^bulk_out 02 (01030b000004[0-9a-f]{4})+(010505000000cd06)? -> [0-9]+$/bulk_out 02 POLLS \2/'
    NO_REQUEST_BUG="a capture sends no request at the pinned commit: libsigrok 524ec154 (2025, Synchronize read and write operations) deleted maynuo_m97_capture_start, which since ffb580cf (2015) sent four registers from U at the start and after every read. dev_acquisition_start now only adds a source (api.c:376-383), and maynuo_m97_receive_data still reads with an address of -1 (protocol.c:227), the reply of a request sent before (modbus.c:437-438, 461-465). So nothing comes and no sample is ever taken: --samples never ends (no limit is reached, protocol.c:244), --time ends with nothing. oxrok sends the frame the commit deleted"
    NO_CLOSE_BUG="sigrok-cli's logger exits on a critical (main.c:46-47), before its sr_dev_close (main.c:130 then 201, session.c:708), so an error leaves PC1 on and the panel locked; oxrok clears it"
    ORDER_BUG="sigrok-cli sets -c's options in hash table order (session.c:700-701) and each -c in turn, here the input before its setpoint; oxrok sends enabled=on last, in a capture just before the start"
    LOOSE_BUG="sigrok-cli takes what is no value (session.c:570's strtof) and sends what it makes of it, here 0; oxrok refuses it and sends nothing"
    NUMBER_BUG="config_set has no check (api.c:320-327) and a NaN or an infinity goes to the load as a float; oxrok refuses what is no number and sends nothing"
    PARTIAL_BUG="sigrok-cli sets -c's options one at a time in hash table order (session.c:700-707), so the input goes on before a later option is refused; oxrok checks them first and leaves it off"
    PURGE_BUG="modbus_serial_rtu_read_begin takes what a bad reply left (modbus_serial_rtu.c:114-116) and nothing clears the rest, so the next reply fails; oxrok drops the input queue before a request"
    ADDR_BUG="modbus_serial_rtu_dev_inst_new keeps the low byte of modbusaddr (modbus_serial_rtu.c:47), so 257 talks to slave 1; oxrok refuses it"
    CHANNEL_WORDS="$NO_CLOSE_BUG; and the words are each CLI's own, with oxrok's prefix on the error it returns"
    OPTION_WORDS="$NO_CLOSE_BUG; and the words are each CLI's own: oxrok says it looked among the device's options"
    SIGINT_BUG="sigrok-cli has no SIGINT handler, it dies where oxrok ends the capture"
    OFF_FIRST="sigrok-cli's hash table order has the setpoint first; oxrok sends enabled=off first of all (docs/CLI.md), so nothing moves under a load that is on"
    ANYKEY="sigrok-cli stops a --continuous at stdin's end (anykey.c), which here is at once, and has taken no sample (see NO_REQUEST_BUG); oxrok has no such stop, so the time limit ends it"

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
    emu_case "scan model commented out" --stderr --filter "$F" --keep-header -- -d $M --scan
    emu_options setters=0 id=9999
    emu_case "scan unknown model" --stderr --filter "$F" --keep-header -- -d $M --scan
    emu_options setters=0
    emu_case "no conn" --keep-header --no-output -- -d maynuo-m97 --scan
    emu_case "no such node" --keep-header --no-output -- -d maynuo-m97:conn=/dev/ttyACM97 --scan

    # The line and the address.
    emu_case "serialcomm misset" --keep-header --no-output -- -d $M:serialcomm=19200/8n1 --scan
    emu_case "serialcomm as it is" --keep-header -- -d $M:serialcomm=9600/8n1 --scan
    emu_case "serialcomm no such" --tx-filter '/^ctl_out/d' --keep-header --no-output -- -d $M:serialcomm=fast --scan
    emu_options setters=0 uart=19200/8n1
    emu_case "serialcomm on a changed line" --keep-header -- -d $M:serialcomm=19200/8n1 --scan
    emu_options setters=0 addr=5
    emu_case "modbusaddr of the slave" --keep-header -- -d $M:modbusaddr=5 --scan
    emu_case "modbusaddr default misses it" --keep-header --no-output -- -d $M --scan
    emu_options setters=0
    emu_case "modbusaddr other" --keep-header --no-output -- -d $M:modbusaddr=7 --scan
    emu_case "modbusaddr 0 is a broadcast" --keep-header --no-output -- -d $M:modbusaddr=0 --scan
    emu_case "modbusaddr over 8 bits" --oracle-differs "$ADDR_BUG" --expect-fail --stderr --filter "$F" -- -d $M:modbusaddr=257 --scan

    # What a slave can answer in the probe, with no second try.
    for code in 01 02 03 04 05 06 08 0a 0b 42; do
        emu_options setters=0 faults=1:exc$code
        emu_case "scan exception $code" --stderr --filter "$F" --keep-header -- -d $M --scan
    done
    for kind in silent addr count func first nocrc; do
        emu_options setters=0 faults=1:$kind
        emu_case "scan $kind" --keep-header --no-output -- -d $M --scan
    done
    emu_options setters=0 faults=1:crc
    emu_case "scan crc" --stderr --filter "$F" --keep-header -- -d $M --scan
    emu_options setters=0 faults=1:short
    emu_case "scan short" --stderr --filter "$F" --keep-header -- -d $M --scan
    emu_options setters=0 faults=1:extra
    emu_case "scan two bytes too many" --keep-header -- -d $M --scan

    # --show: what a group lists, and what the device does.
    emu_options setters=0
    emu_case "show" --keep-header -- -d $M --show
    emu_case "show group" --keep-header -- -d $M -g 1 --show
    emu_case "show no such group" --tx-filter "$P" --stderr --filter "$F" --expect-fail --keep-header -- -d $M -g 2 --show
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
    emu_case "get limit_samples in the group" --tx-filter "$P" --stderr --filter "$F" --expect-fail -- -d $M -g 1 --get limit_samples
    emu_case "get voltage out of the group" --tx-filter "$P" --stderr --filter "$F" --expect-fail -- -d $M --get voltage
    emu_case "get conn" --tx-filter "$P" --stderr --filter "$F" --expect-fail -- -d $M --get conn
    emu_case "get power unlisted" --tx-filter "$P" --stderr --filter "$F" --expect-fail -- -d $M -g 1 --get power
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
    emu_case "get regulation a low byte of 0" -- -d $M -g 1 --get regulation
    emu_options setters=0 coils=0x525 setmode=2
    emu_case "get regulation unregulated" -- -d $M -g 1 --get regulation
    # A reply that comes a byte per transfer.
    emu_options setters=0 chunk=1
    emu_case "get voltage one byte per transfer" -- -d $M -g 1 --get voltage

    # A get that fails: what is left of the device, each key's read refused.
    for key in voltage voltage_target current current_limit enabled regulation ovp_active \
        ovp_threshold ocp_active ocp_threshold otp_active; do
        emu_options setters=0 faults=3:exc04
        emu_case "get refused $key" --tx-filter "$P" --stderr --filter "$F" --expect-fail -- -d $M -g 1 --get $key
    done
    emu_options setters=0 faults=4:exc04
    emu_case "get regulation, the mode refused" --tx-filter "$P" --stderr --filter "$F" --expect-fail -- -d $M -g 1 --get regulation
    for kind in silent crc addr short count func first nocrc; do
        emu_options setters=0 faults=3:$kind
        emu_case "get voltage $kind" --tx-filter "$P" --stderr --filter "$F" --expect-fail -- -d $M -g 1 --get voltage
        emu_case "get enabled $kind" --tx-filter "$P" --stderr --filter "$F" --expect-fail -- -d $M -g 1 --get enabled
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
    emu_case "set enabled a low byte of 0" --no-output -- -d $M -g 1 -c enabled=on --set
    emu_options
    emu_case "set then read back" -- -d $M -g 1 -c voltage_target=12.5 --get voltage_target
    emu_case "set a float then read back" -- -d $M -g 1 -c current_limit=3.3 --get current_limit
    emu_case "set enabled then read back" -- -d $M -g 1 -c enabled=on --get enabled
    # What the list does not hold. A float goes out whatever it is.
    emu_case "set voltage_target over the list" --no-output -- -d $M -g 1 -c voltage_target=151 --set
    emu_case "set voltage_target negative" --no-output -- -d $M -g 1 -c voltage_target=-1 --set
    emu_case "set current_limit over the list" --no-output -- -d $M -g 1 -c current_limit=31 --set
    emu_case "set voltage_target nan" --oracle-differs "$NUMBER_BUG" --expect-fail --stderr --filter "$F" -- -d $M -g 1 -c voltage_target=nan --set
    emu_case "set ocp_threshold inf" --oracle-differs "$NUMBER_BUG" --expect-fail --stderr --filter "$F" -- -d $M -g 1 -c ocp_threshold=inf --set
    emu_case "set ovp_threshold negative" --no-output -- -d $M -g 1 -c ovp_threshold=-1 --set
    emu_case "set voltage_target no number" --oracle-differs "$LOOSE_BUG" --expect-fail --stderr --filter "$F" -- -d $M -g 1 -c voltage_target=abc --set
    # What cannot be set, or not there.
    emu_case "set voltage read only" --tx-filter "$P" --stderr --filter "$F" --expect-fail -- -d $M -g 1 -c voltage=5 --set
    emu_case "set voltage_target out of the group" --tx-filter "$P" --stderr --filter "$F" --expect-fail -- -d $M -c voltage_target=5 --set
    emu_case "set limit_samples in the group" --tx-filter "$P" --stderr --filter "$F" --expect-fail -- -d $M -g 1 -c limit_samples=5 --set
    emu_case "set limit_samples" --no-output -- -d $M -c limit_samples=5 --set
    emu_case "set no such key" --tx-filter "$P" --stderr --filter "$F" --expect-fail -- -d $M -g 1 -c volage_target=5 --set
    # The order on the wire: the setpoint and the input.
    emu_case "set the input after a setpoint" --no-output -- -d $M -g 1 -c current_limit=1:enabled=on --set
    emu_case "set the input before a setpoint" --oracle-differs "$ORDER_BUG" --no-output -- -d $M -g 1 -c enabled=on:current_limit=1 --set
    emu_case "set all the setpoints" --oracle-differs "$ORDER_BUG" --no-output -- -d $M -g 1 -c enabled=on:current_limit=1:voltage_target=5:ovp_threshold=6:ocp_threshold=2 --set
    emu_case "set each -c, input first" --oracle-differs "$ORDER_BUG" --no-output -- -d $M -g 1 -c enabled=on -c current_limit=1 --set
    emu_case "set the input off first" --oracle-differs "$OFF_FIRST" --no-output -- -d $M -g 1 -c current_limit=1:enabled=off --set
    emu_case "the input with an unknown option" --oracle-bug "$PARTIAL_BUG" --stderr --filter "$F" --expect-fail -- -d $M -g 1 -c enabled=on:volage_target=5 --set
    emu_case "the input with a value that is no number" --oracle-differs "$LOOSE_BUG" --expect-fail --stderr --filter "$F" -- -d $M -g 1 -c enabled=on:current_limit=abc --set

    # Writes refused. Each of the input's three steps.
    for set in voltage_target=5 current_limit=1 ovp_threshold=45 ocp_threshold=5; do
        emu_options faults=3:exc04
        emu_case "set refused ${set%%=*}" --tx-filter "$P" --stderr --filter "$F" --expect-fail -- -d $M -g 1 -c $set --set
    done
    for step in 3 4 5; do
        emu_options faults=$step:exc04
        emu_case "set enabled refused at step $step" --tx-filter "$P" --stderr --filter "$F" --expect-fail -- -d $M -g 1 -c enabled=on --set
    done
    for kind in silent crc count short; do
        emu_options faults=3:$kind
        emu_case "set $kind" --tx-filter "$P" --stderr --filter "$F" --expect-fail -- -d $M -g 1 -c current_limit=1 --set
    done

    # PC1, which nothing looks at the answer to.
    emu_options setters=0 faults=2:exc04
    emu_case "open: PC1 refused" -- -d $M -g 1 --get voltage
    emu_options setters=0 faults=2:silent
    emu_case "open: PC1 no answer" -- -d $M -g 1 --get voltage
    emu_options setters=0 faults=2:count
    emu_case "open: PC1 a wrong echo" -- -d $M -g 1 --get voltage
    emu_options setters=0 faults=4:exc04
    emu_case "close: PC1 refused" -- -d $M -g 1 --get voltage
    emu_options setters=0 faults=4:silent
    emu_case "close: PC1 no answer" -- -d $M -g 1 --get voltage
    # What a bad reply leaves fails the next, where nothing clears it.
    emu_options setters=0 faults=2:extra
    emu_case "open: PC1 two bytes too many" --oracle-bug "$PURGE_BUG" -- -d $M -g 1 --get voltage

    # The ways out: the .tx's last write is PC1 off, or is not.
    emu_options setters=0
    emu_case "way out: no number for the samples" --oracle-bug "$NO_CLOSE_BUG" --stderr --filter "$F" --expect-fail -- -d $M --samples abc
    emu_case "way out: no such channel" --oracle-bug "$CHANNEL_WORDS" --stderr --filter "$F" --expect-fail -- -d $M -C Z --samples 1
    emu_case "way out: no such output" --oracle-bug "$NO_CLOSE_BUG" --stderr --filter "$F" --expect-fail -- -d $M -O nosuch --samples 1
    emu_case "way out: get no such key" --oracle-bug "$OPTION_WORDS" --stderr --filter "$F" --expect-fail -- -d $M -g 1 --get nosuch

    emu_options setters=0 faults=3:exc04
    emu_case "way out: a get refused" --oracle-bug "$NO_CLOSE_BUG" --stderr --filter "$F" --expect-fail -- -d $M -g 1 --get voltage
    emu_options faults=3:exc04
    emu_case "way out: a set refused" --oracle-bug "$NO_CLOSE_BUG" --stderr --filter "$F" --expect-fail -- -d $M -g 1 -c current_limit=1 --set

    # A capture sends no request (api.c's start, protocol.c's read of a reply),
    # so it takes no sample: only a time limit ends it. oxrok asks.
    emu_options setters=0 u=24.5,25,26 i=2.5,2.75,3 answers=5
    emu_case "time" --oracle-differs "$NO_REQUEST_BUG" --tx-filter "$N" -- -d $M --time 700ms
    emu_case "continuous and a time limit" --oracle-differs "$ANYKEY" --tx-filter "$N" -- -d $M --continuous --time 700ms
    emu_options setters=0 u=24.5,25,26 i=2.5,2.75,3
    emu_case "samples" --oracle-bug "$NO_REQUEST_BUG" --oracle-hangs --timeout 3 -- -d $M --samples 3
    emu_case "samples analog" --oracle-bug "$NO_REQUEST_BUG" --oracle-hangs --timeout 3 -- -d $M --samples 3 -O analog
    emu_case "samples csv" --oracle-bug "$NO_REQUEST_BUG" --oracle-hangs --timeout 3 -- -d $M --samples 3 -O csv:header=false:time=false
    # The C's side of those alone, to the end of what it sends.
    emu_case "lib samples, none ever come" --libsigrok --hangs --timeout 3 --no-output -- -d $M --samples 1
    emu_options setters=0 answers=3 quiet=50
    emu_case "unplug in a capture" --oracle-bug "$NO_REQUEST_BUG" --oracle-hangs --timeout 10 --unplug model -- -d $M --samples 3
    emu_options setters=0 answers=5 quiet=50
    emu_case "interrupt" --oracle-bug "$SIGINT_BUG" --timeout 10 --interrupt model --tx-filter "$N" -- -d $M --samples 1000
    # Answers gone wrong in a capture: no sample, and the next read asks again.
    for kind in silent crc short count func exc04; do
        emu_options setters=0 faults=3:$kind
        emu_case "a poll lost: $kind" --oracle-bug "$NO_REQUEST_BUG" --oracle-hangs --timeout 3 -- -d $M --samples 1
    done
    emu_options setters=0 faults=3:extra
    emu_case "a poll with two bytes too many" --oracle-bug "$NO_REQUEST_BUG" --oracle-hangs --timeout 3 -- -d $M --samples 2
    # A load that goes quiet with only a sample limit: 5 s, then the end.
    emu_options setters=0 answers=2
    emu_case "silent in a capture" --oracle-bug "$NO_REQUEST_BUG" --oracle-hangs --timeout 15 --tx-filter "$N" --no-output -- -d $M --samples 2
    emu_options setters=0 answers=3
    emu_case "silent after a sample" --oracle-bug "$NO_REQUEST_BUG" --oracle-hangs --timeout 15 --tx-filter "$N" -- -d $M --samples 2
    # Two reads lost under a time limit, then the readings again.
    emu_options setters=0 faults=3:silent,4:silent answers=6
    emu_case "time recovers after failed reads" --oracle-differs "$NO_REQUEST_BUG" --tx-filter "$N" -- -d $M --time 2s
    # What C does with a reply that came all the same, from a slave no one has:
    # this and no more shows C's frames, see unasked= in the model. oxrok's request
    # purges them, so these hold C's frames to the frames of oxrok's polls.
    emu_options setters=0 unasked=3 u=12.5,13,0.0001 i=1.25,2,0.0001
    emu_case "unasked samples 3" --tx-filter "$T" -- -d $M --samples 3
    emu_case "unasked analog" --tx-filter "$T" -- -d $M --samples 3 -O analog
    emu_case "unasked csv" --tx-filter "$T" -- -d $M --samples 3 -O csv:header=false:time=false
    emu_case "unasked V1 only" --tx-filter "$T" -- -d $M -C V1 --samples 3
    emu_case "unasked I1 only" --tx-filter "$T" -- -d $M -C I1 --samples 3
    emu_case "config limit_samples 0" --tx-filter "$P" --stderr --filter "$F" --expect-fail -- -d $M -c limit_samples=0 --samples 1
    emu_options setters=0 unasked=4 u=0,150,1234.5678,-1 i=0,30,0.00012345,-1
    emu_case "unasked corners" --tx-filter "$T" -- -d $M --samples 4
    emu_options setters=0 unasked=3 u=inf,-inf,nan i=nan,inf,-inf
    emu_case "unasked no number" --tx-filter "$T" -- -d $M --samples 3
    # A setting before a capture: its frames, then nothing. A reply no one asked
    # for would only meet the setting's read, so these have none.
    emu_options ifix=3
    emu_case "capture with the input on" --oracle-bug "$NO_REQUEST_BUG" --oracle-hangs --timeout 3 -- -d $M -g 1 -c enabled=on --samples 2
    emu_case "capture with setpoints and the input" --oracle-bug "$NO_REQUEST_BUG" --oracle-hangs --timeout 3 -- -d $M -g 1 -c current_limit=2:enabled=on --samples 2
    # More came than the limit: what is left fails PC1 off's reply, and that is
    # not looked at.
    emu_options setters=0 unasked=5
    emu_case "unasked more than the limit" --tx-filter "$T" -- -d $M --samples 2
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
