# Siglent SDL1000X electronic loads (emu/src/models/siglent-sdl10x0.rs), on USBTMC and
# on TCP. See docs/USB.md, "Writing a SCPI model".
EMU_MODEL=siglent-sdl10x0
EMU_TMC=1
EMU_TCP=1
EMU_DRIVER=siglent-sdl10x0
EMU_FULL=$EMU_DRIVER
EMU_GCOV=siglent-sdl10x0
EMU_GCOV_SHARED="src/scpi/scpi.c src/scpi/scpi_usbtmc_libusb.c src/scpi/scpi_tcp.c src/tcp.c"

emu_cases() {
    U=siglent-sdl10x0
    T=siglent-sdl10x0:conn=tcp-raw/127.0.0.1/$EMU_TCP_PORT
    # The capture's first lines: how many come after them is the clock's.
    HEAD='7,$d'
    TXHEAD='13,$d'
    # In --show, which sigrok-cli leaves blank: "show keeps the threshold" holds it.
    NO_OCP='/ocp_threshold:/d'

    OCP="api.c:265 reads the current's protection level with sr_scpi_get_bool, where the manual has a number (PDF 58), so it fails on 8.000, and on a 1 returns a float it never set (api.c:268); oxrok reads the number"
    LIST="siglent_sdl10x0_string_to_mode (protocol.c:57) knows CC, CV, CP and CR, while config_list and the load's own answer give CURRENT and the rest, so C refuses what it lists; oxrok takes both"
    UNKNOWN="string_to_mode's loop reaches LED, whose name is Unknown (protocol.c:35-36, 62), so C sends :FUNC Unknown; oxrok refuses it"
    NAN="config_set prints what it's given (api.c:317), so C sends :VOLT:LEV:IMM nan; oxrok refuses it"
    ORDER="docs/CLI.md: -c holds enabled=on until the setpoints went out, where sigrok-cli sends them as given"
    SOURCE="protocol.c:169-173 removes the source at a callback with no G_IO_IN, so 100 ms with no answer ends C's capture, with exit 0 and what it has; oxrok waits for the answer"
    USB="usb_source_add's callback has no fd a read would wake, as no transfer is in flight, so C's capture sends one question, ends after 100 ms by protocol.c:169-173, and exits 0 with nothing; oxrok captures"
    BAD="receive_data returns on an answer sr_atof_ascii refuses, with no next question, so the source's timeout ends C's capture with exit 0; oxrok drains the line and asks the round again, three times"
    ATOF="sr_atof_ascii gives 0 for nothing and takes nan, inf and hexadecimal, and 1e40 is inf as a float, each a reading with exit 0; oxrok takes what the manual shows as a number, and asks again"
    TCP_BLOCKS="scpi_tcp_raw_read_data has sr_tcp_read_bytes block, and its recv() has no timeout, so sr_scpi_get_string's 1 s never runs and a silent load holds sigrok-cli for ever; oxrok's read has the timeout"
    SIGPIPE="sr_tcp_write_bytes is a plain send() and nothing ignores SIGPIPE: at a load's close C's read gets nothing, and the next question kills sigrok-cli (exit 141); oxrok's capture fails"
    SIGINT_BUG="sigrok-cli has no SIGINT handler, so it dies at the signal; oxrok ends its capture and closes"

    emu_case "scan" --keep-header -- -d $U --scan
    emu_case "scan tcp" --keep-header -- -d $T --scan
    emu_case "show" --keep-header -- -d $U --show
    emu_case "show tcp" --keep-header -- -d $T --show
    emu_case "show group" --keep-header --filter "$NO_OCP" -- -d $T -g 1 --show
    emu_case "show group usbtmc" --keep-header --filter "$NO_OCP" -- -d $U -g 1 --show
    for k in enabled regulation voltage voltage_target current current_limit power power_target resistance resistance_target opp_enabled opp_active opp_threshold ocp_enabled ocp_active; do
        emu_case "get $k" -- -d $T -g 1 --get $k
    done
    emu_case "show keeps the threshold" --keep-header --oracle-differs "$OCP" -- -d $T -g 1 --show
    emu_case "get voltage usbtmc" --record -- -d $U -g 1 --get voltage
    emu_case "get ocp_threshold" --oracle-bug "$OCP" -- -d $T -g 1 --get ocp_threshold
    emu_case "get limit_samples" -- -d $T --get limit_samples
    emu_case "get limit_time" -- -d $T --get limit_time
    emu_case "get voltage without the group" --expect-fail --no-output -- -d $T --get voltage
    emu_case "get limit_time in the group" --expect-fail --no-output -- -d $T -g 1 --get limit_time

    # A setting is a line, with no answer.
    emu_case "set enabled on" --no-output -- -d $T -g 1 -c enabled=on --set
    emu_case "set enabled off" --no-output -- -d $T -g 1 -c enabled=off --set
    emu_case "set enabled usbtmc" --no-output -- -d $U -g 1 -c enabled=on --set
    emu_case "set regulation CC" --no-output -- -d $T -g 1 -c regulation=CC --set
    emu_case "set regulation CV" --no-output -- -d $T -g 1 -c regulation=CV --set
    emu_case "set regulation CP" --no-output -- -d $T -g 1 -c regulation=CP --set
    emu_case "set regulation CR" --no-output -- -d $T -g 1 -c regulation=CR --set
    emu_case "set regulation by its start" --no-output -- -d $T -g 1 -c regulation=CCX --set
    emu_case "set regulation CURRENT" --no-output --oracle-bug "$LIST" -- -d $T -g 1 -c regulation=CURRENT --set
    emu_case "set regulation Unknown" --no-output --expect-fail --oracle-differs "$UNKNOWN" -- -d $T -g 1 -c regulation=Unknown --set
    emu_case "set regulation no such" --no-output --expect-fail -- -d $T -g 1 -c regulation=XYZ --set
    emu_case "set voltage_target" --no-output -- -d $T -g 1 -c voltage_target=5.125 --set
    emu_case "set current_limit" --no-output -- -d $T -g 1 -c current_limit=2.5 --set
    emu_case "set power_target" --no-output -- -d $T -g 1 -c power_target=100 --set
    emu_case "set resistance_target" --no-output -- -d $T -g 1 -c resistance_target=47.5 --set
    emu_case "set opp_threshold" --no-output -- -d $T -g 1 -c opp_threshold=150 --set
    emu_case "set ocp_threshold" --no-output -- -d $T -g 1 -c ocp_threshold=7.25 --set
    emu_case "set voltage rounds" --no-output -- -d $T -g 1 -c voltage_target=1.23456 --set
    emu_case "set voltage out of range" --no-output -- -d $T -g 1 -c voltage_target=200 --set
    emu_case "says a number is off the list" --stderr --oracle-differs "oxrok warns, as a load's refusal goes unheard; C says nothing" -- -d $T -g 1 -c voltage_target=200 --set
    emu_case "set voltage nan" --no-output --expect-fail --oracle-differs "$NAN" -- -d $T -g 1 -c voltage_target=nan --set
    emu_case "set then capture" -- -d $T -g 1 -c current_limit=2.5:regulation=CC --samples 1 -O analog
    emu_case "enabled last" -- -d $T -g 1 -c current_limit=2.5:enabled=on --samples 2 -O analog
    emu_case "enabled first" --oracle-differs "$ORDER" -- -d $T -g 1 -c enabled=on:current_limit=2.5 --samples 2 -O analog

    # A capture: the request after the last answer stays out, unread.
    emu_case "samples" -- -d $T --samples 3 -O analog
    emu_case "one sample" -- -d $T --samples 1 -O analog
    emu_case "samples by default output" -- -d $T --samples 2
    emu_case "time" --filter "$HEAD" --tx-filter "$TXHEAD" -- -d $T --time 100 -O analog
    # The rate a real load might have: C's source has 100 ms.
    emu_options pace=2
    emu_case "real rate samples" -- -d $T --samples 2 -O analog
    emu_case "real rate time" --filter '3,$d' --tx-filter '9,$d' -- -d $T --time 150 -O analog
    emu_options pace=10
    emu_case "slow load" --oracle-differs "$SOURCE" -- -d $T --samples 1 -O analog
    emu_case "slow load time" --oracle-differs "$SOURCE" -- -d $T --time 50 -O analog
    emu_options late-at=2:10
    emu_case "first answer late" --oracle-differs "$SOURCE" -- -d $T --samples 2 -O analog
    emu_options late-at=4:10
    emu_case "answer late in a capture" --oracle-differs "$SOURCE" -- -d $T --samples 2 -O analog

    # What comes back.
    emu_options volt=-0.5,1.5e1,+3
    emu_case "other forms of a number" -- -d $T --samples 3 -O analog
    emu_options eol=crlf
    emu_case "crlf" -- -d $T --samples 2 -O analog
    emu_options eol=none
    emu_case "no line end" -- -d $T --samples 2 -O analog
    emu_options volt=abc
    emu_case "volt not a number" --oracle-differs "$BAD" -- -d $T --samples 2 -O analog
    emu_options curr=abc
    emu_case "curr not a number" --oracle-differs "$BAD" -- -d $T --samples 2 -O analog
    emu_options volt=abc,abc,abc,abc,abc,abc
    emu_case "volt never a number" --no-output --oracle-differs "$BAD" --oxrok-fails "No usable answer from the load" -- -d $T --samples 2 -O analog
    emu_options curr=abc,abc,abc,abc,abc,abc
    emu_case "curr never a number" --no-output --oracle-differs "$BAD" --oxrok-fails "No usable answer from the load" -- -d $T --samples 2 -O analog
    emu_options volt=,5
    emu_case "empty answer" --oracle-differs "$ATOF" -- -d $T --samples 2 -O analog
    emu_options volt=nan,5
    emu_case "nan" --oracle-differs "$ATOF" -- -d $T --samples 2 -O analog
    emu_options volt=inf,5
    emu_case "inf" --oracle-differs "$ATOF" -- -d $T --samples 2 -O analog
    emu_options volt=0x10,5
    emu_case "hex" --oracle-differs "$ATOF" -- -d $T --samples 2 -O analog
    emu_options volt=1e40,5
    emu_case "beyond a float" --oracle-differs "$ATOF" -- -d $T --samples 2 -O analog
    emu_options volt=1V,5
    emu_case "a unit behind" --oracle-differs "$BAD" -- -d $T --samples 2 -O analog
    emu_options volt=
    emu_case "get empty" --no-output --expect-fail --oracle-differs "$ATOF" -- -d $T -g 1 --get voltage
    emu_options volt=abc
    emu_case "get not a number" --no-output --expect-fail -- -d $T -g 1 --get voltage
    emu_options input=maybe
    emu_case "get not a boolean" --no-output --expect-fail -- -d $T -g 1 --get enabled
    emu_options input=ON
    emu_case "get enabled word" -- -d $T -g 1 --get enabled
    emu_options func=LED
    emu_case "get regulation led" -- -d $T -g 1 --get regulation
    emu_options ocp-level=1
    # C reads a boolean here and returns a float it never set, which changes per run.
    emu_case "get ocp_threshold of 1" --oracle-differs "$OCP" --filter '/^1\.0$/!s/.*/what the stack held/' -- -d $T -g 1 --get ocp_threshold
    emu_options opp-state=0 ocp-state=1
    emu_case "protection states" -- -d $T -g 1 --get ocp_active
    emu_options volt=abc
    emu_case "show with a bad reading" --keep-header --filter "$NO_OCP" -- -d $T -g 1 --show

    # Who it is.
    emu_options model=SDL1030X-E
    emu_case "sdl1030x-e show" --keep-header --filter "$NO_OCP" -- -d $T -g 1 --show
    emu_options idn=Acme,SDL1020X,1,2
    emu_case "another maker" --no-output -- -d $T --scan
    emu_options name=SDL2000X
    emu_case "another model" --no-output -- -d $T --scan
    emu_options idn=short
    emu_case "no idn to speak of" --no-output -- -d $U --scan
    emu_options
    emu_case "usbtmc scan names it" --keep-header -- -d $U:conn=usbtmc --scan
    emu_reset
    emu_case "usbtmc by address" --keep-header -- -d $U:conn=usbtmc/$EMU_USB --scan
    emu_case "tcp nothing listens" --no-output -- -d $U:conn=tcp-raw/127.0.0.1/1 --scan

    # When it says nothing.
    emu_options mute-at=1
    emu_case "scan silent" --no-output -- -d $U --scan
    emu_case "scan silent tcp" --no-output --oracle-bug "$TCP_BLOCKS" --oracle-hangs --timeout 4 -- -d $T --scan
    emu_options silent-at=2
    emu_case "get silent" --no-output --expect-fail -- -d $U -g 1 --get voltage
    emu_options mute-at=2
    emu_case "get silent tcp" --no-output --expect-fail --oracle-bug "$TCP_BLOCKS" --oracle-hangs --timeout 4 -- -d $T -g 1 --get voltage
    # 10 s, the one run of that bound in full: the rest is on a moved clock in the unit tests.
    emu_case "capture silent" --no-output --oxrok-fails "No answer from the load in 10 s" -- -d $T --samples 2 -O analog
    emu_case "capture silent time" --no-output -- -d $T --time 100 -O analog
    emu_case "says no reading came" --stderr --oracle-differs "oxrok warns, where C's capture of nothing says nothing" -- -d $T --time 100 -O analog
    emu_options mute-at=4
    emu_case "goes silent time" -- -d $T --time 300 -O analog
    emu_case "says it went silent" --stderr --oracle-differs "oxrok warns, where C's capture says nothing" -- -d $T --time 300 -O analog
    emu_options late-at=2:5
    emu_case "scan late tcp" --keep-header -- -d $T -g 1 --get voltage

    # USBTMC faults on a get. A capture there sends one request and is over.
    emu_options empty-at=1
    emu_case "usbtmc empty transfer" --keep-header -- -d $U --scan
    emu_options message=4
    emu_case "usbtmc message in pieces" -- -d $U -g 1 --get voltage
    emu_options stall=caps
    emu_case "usbtmc capabilities stalled" --keep-header -- -d $U --scan
    emu_options bad-tag-at=1
    emu_case "usbtmc wrong tag" --no-output -- -d $U --scan
    emu_options in-stall=1
    emu_case "usbtmc bulk in stalled" --no-output --expect-fail -- -d $U -g 1 --get voltage
    emu_options
    # Every capture on USBTMC differs: stock C has none to compare.
    emu_case "usbtmc capture" --oracle-differs "$USB" -- -d $U --samples 3 -O analog
    emu_case "usbtmc capture time" --oracle-differs "$USB" --filter '5,$d' --tx-filter '28,$d' -- -d $U --time 1000 -O analog
    emu_options late-at=2:5
    emu_case "usbtmc late answer" --oracle-differs "$USB" -- -d $U --samples 1 -O analog
    emu_options mute-at=2
    emu_case "usbtmc silent capture" --no-output --oracle-differs "$USB" --tx-filter '20,$d' -- -d $U --time 100 -O analog
    # At the one question C asks: the same .tx, and C has no read to fail.
    emu_options mark-at=2
    emu_case "usbtmc unplug" --no-output --oxrok-fails "unplug" --unplug model -- -d $U --samples 9 -O analog
    emu_options

    # An unplug. On TCP the instrument closes the connection.
    emu_case "unplug in a capture tcp" --oracle-bug "$SIGPIPE" --oxrok-fails "closed" --unplug tcp-in=5 -- -d $T --samples 9 -O analog
    emu_case "unplug before a capture tcp" --no-output --oracle-bug "$SIGPIPE" --oxrok-fails "closed" --unplug tcp-in=1 -- -d $T --samples 9 -O analog
    emu_case "unplug in a get tcp" --no-output --expect-fail --unplug tcp-in=1 -- -d $T -g 1 --get voltage

    # A stop, well into a capture that goes on.
    emu_options pace=2 mark-at=6
    emu_case "interrupt" --oracle-bug "$SIGINT_BUG" --interrupt tcp-model --timeout 15 --filter '5,$d' --tx-filter '/^interrupt$/q' -- -d $T --samples 900 -O analog
    emu_options
}
