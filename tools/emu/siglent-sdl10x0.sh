# Siglent SDL1000X electronic loads (emu/src/models/siglent-sdl10x0.rs), on USBTMC and
# on TCP. See docs/USB.md, "Writing a SCPI model".
EMU_MODEL=siglent-sdl10x0
EMU_TMC=1
EMU_TCP=1
EMU_DRIVER=siglent-sdl10x0
EMU_GCOV=siglent-sdl10x0
EMU_GCOV_SHARED="src/scpi/scpi.c src/scpi/scpi_usbtmc_libusb.c src/scpi/scpi_tcp.c src/tcp.c"

emu_cases() {
    U=siglent-sdl10x0
    T=siglent-sdl10x0:conn=tcp-raw/127.0.0.1/$EMU_TCP_PORT
    # The capture's first lines: how many come after them is the clock's.
    HEAD='7,$d'
    TXHEAD='13,$d'

    emu_case "scan" --keep-header -- -d $U --scan
    emu_case "scan tcp" --keep-header -- -d $T --scan
    emu_case "show" --keep-header -- -d $U --show
    emu_case "show tcp" --keep-header -- -d $T --show
    emu_case "show group" --keep-header -- -d $T -g 1 --show
    emu_case "show group usbtmc" --keep-header -- -d $U -g 1 --show
    for k in enabled regulation voltage voltage_target current current_limit power power_target resistance resistance_target opp_enabled opp_active opp_threshold ocp_enabled ocp_active; do
        emu_case "get $k" -- -d $T -g 1 --get $k
    done
    emu_case "get voltage usbtmc" -- -d $U -g 1 --get voltage
    emu_case "get ocp_threshold" --expect-fail --no-output -- -d $T -g 1 --get ocp_threshold
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
    emu_case "set regulation CURRENT" --no-output --expect-fail -- -d $T -g 1 -c regulation=CURRENT --set
    emu_case "set regulation Unknown" --no-output -- -d $T -g 1 -c regulation=Unknown --set
    emu_case "set regulation no such" --no-output --expect-fail -- -d $T -g 1 -c regulation=XYZ --set
    emu_case "set voltage_target" --no-output -- -d $T -g 1 -c voltage_target=5.125 --set
    emu_case "set current_limit" --no-output -- -d $T -g 1 -c current_limit=2.5 --set
    emu_case "set power_target" --no-output -- -d $T -g 1 -c power_target=100 --set
    emu_case "set resistance_target" --no-output -- -d $T -g 1 -c resistance_target=47.5 --set
    emu_case "set opp_threshold" --no-output -- -d $T -g 1 -c opp_threshold=150 --set
    emu_case "set ocp_threshold" --no-output -- -d $T -g 1 -c ocp_threshold=7.25 --set
    emu_case "set voltage rounds" --no-output -- -d $T -g 1 -c voltage_target=1.23456 --set
    emu_case "set voltage out of range" --no-output -- -d $T -g 1 -c voltage_target=200 --set
    emu_case "set voltage nan" --no-output -- -d $T -g 1 -c voltage_target=nan --set
    emu_case "set then capture" -- -d $T -g 1 -c current_limit=2.5:regulation=CC --samples 1 -O analog
    emu_case "enabled last" -- -d $T -g 1 -c current_limit=2.5:enabled=on --samples 2 -O analog
    emu_case "enabled first" -- -d $T -g 1 -c enabled=on:current_limit=2.5 --samples 2 -O analog

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
    emu_case "slow load" --no-output -- -d $T --samples 1 -O analog
    emu_case "slow load time" --no-output -- -d $T --time 50 -O analog
    emu_options late-at=2:10
    emu_case "first answer late" --no-output -- -d $T --samples 2 -O analog
    emu_options late-at=4:10
    emu_case "answer late in a capture" -- -d $T --samples 2 -O analog

    # What comes back.
    emu_options volt=-0.5,1.5e1,+3
    emu_case "other forms of a number" -- -d $T --samples 3 -O analog
    emu_options eol=crlf
    emu_case "crlf" -- -d $T --samples 2 -O analog
    emu_options eol=none
    emu_case "no line end" -- -d $T --samples 2 -O analog
    emu_options volt=abc
    emu_case "volt not a number" --no-output -- -d $T --samples 2 -O analog
    emu_options curr=abc
    emu_case "curr not a number" --no-output -- -d $T --samples 2 -O analog
    emu_options volt=abc,abc,abc,abc,abc,abc
    emu_case "volt never a number" --no-output -- -d $T --samples 2 -O analog
    emu_options curr=abc,abc,abc,abc,abc,abc
    emu_case "curr never a number" --no-output -- -d $T --samples 2 -O analog
    emu_options volt=,5
    emu_case "empty answer" -- -d $T --samples 2 -O analog
    emu_options volt=nan,5
    emu_case "nan" -- -d $T --samples 2 -O analog
    emu_options volt=inf,5
    emu_case "inf" -- -d $T --samples 2 -O analog
    emu_options volt=0x10,5
    emu_case "hex" -- -d $T --samples 2 -O analog
    emu_options volt=1e40,5
    emu_case "beyond a float" -- -d $T --samples 2 -O analog
    emu_options volt=1V,5
    emu_case "a unit behind" --no-output -- -d $T --samples 2 -O analog
    emu_options volt=
    emu_case "get empty" -- -d $T -g 1 --get voltage
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
    emu_case "get ocp_threshold of 1" --filter '/^1\.0$/!s/.*/what the stack held/' -- -d $T -g 1 --get ocp_threshold
    emu_options opp-state=0 ocp-state=1
    emu_case "protection states" -- -d $T -g 1 --get ocp_active
    emu_options volt=abc
    emu_case "show with a bad reading" --keep-header -- -d $T -g 1 --show

    # Who it is.
    emu_options model=SDL1030X-E
    emu_case "sdl1030x-e show" --keep-header -- -d $T -g 1 --show
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
    emu_case "scan silent tcp" --no-output --hangs --timeout 4 -- -d $T --scan
    emu_options silent-at=2
    emu_case "get silent" --no-output --expect-fail -- -d $U -g 1 --get voltage
    emu_options mute-at=2
    emu_case "get silent tcp" --no-output --hangs --timeout 4 -- -d $T -g 1 --get voltage
    emu_case "capture silent" --no-output -- -d $T --samples 2 -O analog
    emu_case "capture silent time" --no-output -- -d $T --time 100 -O analog
    emu_options mute-at=4
    emu_case "goes silent time" -- -d $T --samples 9 -O analog
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
    emu_case "usbtmc capture" --no-output -- -d $U --samples 3 -O analog
    emu_case "usbtmc capture time" --no-output -- -d $U --time 100 -O analog

    # An unplug. On TCP the instrument closes the connection.
    emu_case "unplug in a capture tcp" --expect-fail --unplug tcp-in=5 -- -d $T --samples 9 -O analog
    emu_case "unplug after a question tcp" --no-output --expect-fail --unplug tcp-in=1 -- -d $T --samples 9 -O analog
    emu_case "unplug in a get tcp" --no-output --expect-fail --unplug tcp-in=1 -- -d $T -g 1 --get voltage
}
