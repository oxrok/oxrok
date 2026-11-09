# The ASIX OMEGA behind ASIX's omegartmcli program (emu/src/models/asix-omega-rtm-cli.rs):
# no device on any bus, but a program both tools run, which oxrok-emu stands in for.
# Sourced by tools/diffcli.sh --emu; see docs/USB.md, "A program as the model".
EMU_MODEL=asix-omega-rtm-cli
# Nothing to attach; the socket is the stand-in's mark for --interrupt tcp-model.
EMU_TCP=only
EMU_STANDIN=omegartmcli
EMU_DRIVER=asix-omega-rtm-cli
EMU_GCOV=asix-omega-rtm-cli
EMU_FULL=$EMU_DRIVER

emu_cases() {
    D=asix-omega-rtm-cli
    # A capture's output in few lines, and through --digest the hash of all of it.
    A='1~64!d'
    L='s/^oxrok: [a-z]+: oxrok::drivers::asix_omega_rtm_cli(::[a-z]+)?: /sr: asix-omega-rtm-cli: /'
    # glib's words for a failed exec name the file and the system's error.
    X="$L
s/^(sr: asix-omega-rtm-cli: Cannot execute RTM CLI process: ).*/\1WHY/"
    SIGINT_BUG="sigrok-cli has no SIGINT handler, it dies where oxrok ends the capture"
    PROBE_NAMES="scan() takes probe_names (c18f4891) but scanopts lists conn alone (api.c:112-114), so sigrok-cli refuses it"
    ONE_READ="omega_rtm_cli_open makes one read() of 6 bytes and fails on fewer (protocol.c:97-106); oxrok takes the chunk in pieces"
    NO_BOUND="omega_rtm_cli_open's read() has no bound (protocol.c:97), so a program that says nothing holds sigrok-cli for ever; oxrok gives up after 30 s"
    HUP="the source asks for G_IO_IN and G_IO_ERR (api.c:369), a closed pipe gives G_IO_HUP, which omega_rtm_cli_receive_data ignores (protocol.c:282-326), so sigrok-cli spins until a time limit, and with a sample limit for ever; oxrok's capture ends"
    ORPHAN="omega_rtm_cli_close closes the pipes and calls g_spawn_close_pid (protocol.c:125-160, 151), which does nothing on Unix; glib's spawn leaves the program to init, which reaps it but kills nothing, so a program that doesn't end by itself outlives sigrok-cli; oxrok kills its process group after 2 s and reaps it"
    ONE_MORE="omega_rtm_cli_process_rawdata submits both samples of a chunk and caps the count after (protocol.c:231-251), so with one sample left it sends two, and sigrok-cli writes them; oxrok sends what the limit says"

    emu_case "scan" --keep-header -- -d $D --scan
    emu_case "show" --keep-header -- -d $D --show
    emu_case "get samplerate" -- -d $D --get samplerate
    emu_case "get limit" -- -d $D --get limit_samples
    emu_case "get time" -- -d $D --get limit_time
    emu_case "get conn none" --stderr --expect-fail -- -d $D --get conn
    emu_case "set limit" -- -d $D -c limit_samples=100 --get limit_samples
    emu_case "set time" -- -d $D -c limit_time=250 --get limit_time
    emu_case "set samplerate" --stderr --expect-fail -- -d $D -c samplerate=100M --set
    emu_case "probe names" --keep-header --oracle-differs "$PROBE_NAMES" -- -d $D:probe_names=a,b,c --scan
    emu_case "probe names capture" --oracle-bug "$PROBE_NAMES" -- -d $D:probe_names=-uart --samples 8 -O bits

    # conn=sn=: four digits get C's 03, six or eight go as they are, other lengths
    # are an error and no -serial, and a conn= without sn= is no device.
    emu_case "conn sn eight" --keep-header -- -d $D:conn=sn=A6031234 --show
    emu_case "conn sn four" -- -d $D:conn=sn=1234 --samples 4 -O bits
    emu_case "conn sn six" -- -d $D:conn=sn=031234 --samples 4 -O bits
    emu_case "conn sn eight capture" -- -d $D:conn=sn=A6031234 --samples 4 -O bits
    emu_case "conn sn another device" --no-output --expect-fail --oxrok-fails "before its first chunk" -- -d $D:conn=sn=A6039999 --samples 4 -O bits
    emu_case "conn sn not hex" --no-output --expect-fail --oxrok-fails "before its first chunk" -- -d $D:conn=sn=zz1234 --samples 4 -O bits
    emu_case "conn sn five" --stderr --filter "$L" -- -d $D:conn=sn=12345 --scan
    emu_case "conn sn five capture" -- -d $D:conn=sn=12345 --samples 4 -O bits
    emu_case "conn sn empty" --keep-header -- -d $D:conn=sn= --scan
    emu_case "conn no sn" --stderr --filter "$L" -- -d $D:conn=031234 --scan

    # The program, and where it is.
    export OMEGARTMCLI="$EMU_STANDIN_DIR/omegartmcli"
    emu_case "named by OMEGARTMCLI" -- -d $D --samples 4 -O bits
    export OMEGARTMCLI=""
    emu_case "empty OMEGARTMCLI" -- -d $D --samples 4 -O bits
    export OMEGARTMCLI=/nonexistent/omegartmcli
    emu_case "no program" --no-output -- -d $D --scan
    export OMEGARTMCLI="$TOOLS/emu/asix-omega-rtm-cli.sh"
    emu_case "not executable" --stderr --filter "$X" -- -d $D --scan
    unset OMEGARTMCLI

    # What -version says.
    emu_standin banner="$(emu_hex 'omegartmcli.exe Omega Real-Time Mode\nVersion 2016-12-14\n')"
    emu_case "version with lf" --keep-header -- -d $D --scan
    emu_standin banner="$(emu_hex 'Version 3.0\rjunk\n')"
    emu_case "version cut at cr" --keep-header -- -d $D --scan
    emu_standin banner="$(emu_hex 'xVersion 1 2 3')"
    emu_case "version anywhere" --keep-header -- -d $D --scan
    emu_standin banner="$(emu_hex 'omegartmcli.exe\r\nversion 2016\r\n')"
    emu_case "no version line" --stderr --filter "$L" -- -d $D --scan
    emu_standin banner="$(emu_hex 'Version \r\n')"
    emu_case "empty version" --stderr --filter "$L" -- -d $D --scan
    emu_standin banner=
    emu_case "no banner" --no-output -- -d $D --scan
    emu_standin version-exit=0
    emu_case "version exits 0" --keep-header -- -d $D --scan
    emu_standin version-exit=2 stderr=100000
    emu_case "version exits 2" --keep-header -- -d $D --scan
    emu_standin

    # Captures.
    emu_case "capture one" --oracle-differs "$ONE_MORE" -- -d $D --samples 1 -O bits
    emu_case "capture two" -- -d $D --samples 2 -O bits
    emu_case "capture three" --oracle-differs "$ONE_MORE" -- -d $D --samples 3 -O bits
    emu_case "capture bits" -- -d $D --samples 64 -O bits
    emu_case "capture hex" --digest --filter "$A" -- -d $D --samples 4096 -O hex
    emu_case "capture csv" --digest --filter "$A" -- -d $D --samples 300 -O csv:header=false
    emu_case "capture binary" --digest --filter "1~64!d" -- -d $D --samples 1000 -O binary
    emu_case "capture channels" -- -d $D -C 1,16 --samples 64 -O bits
    # 13 chunks are 46 samples; the 14th repeats its sample 78 times, then two.
    emu_case "limit in a run" -- -d $D --samples 60 -O hex
    emu_case "limit at a run's end" -- -d $D --samples 124 -O hex
    emu_case "limit after a run" --oracle-differs "$ONE_MORE" -- -d $D --samples 125 -O hex
    emu_case "limit by config" -- -d $D -c limit_samples=10 --samples 10 -O bits
    emu_case "a long capture" --digest --filter "$A" -- -d $D --samples 2M -O hex
    emu_standin stamp=32768 idle=1
    emu_case "idle" --digest --filter "$A" -- -d $D --samples 200000 -O hex
    emu_standin stamp=32768 idle=1 pace=real
    emu_case "idle at the real pace" --digest --filter "$A" -- -d $D --samples 1M -O hex
    emu_standin stamp=1 pace=real
    emu_case "busy at the real pace" --digest --filter "$A" -- -d $D --samples 100000 -O hex
    emu_standin bytes=600 then=quiet
    emu_case "time" --digest --filter "$A" -- -d $D --time 300 -O hex
    emu_standin piece=7 gap=1
    emu_case "pieces of seven" -- -d $D --samples 100 -O bits
    emu_standin piece=6 gap=20
    emu_case "pieces of six" -- -d $D --samples 100 -O bits
    emu_standin stderr=1000000
    emu_case "noise on stderr" -- -d $D --samples 64 -O bits
    emu_standin start=1500
    emu_case "a slow start" -- -d $D --samples 8 -O bits
    emu_standin eof=ignore
    emu_case "stdin ignored" -- -d $D --samples 64 -O bits
    emu_standin

    # A start that fails.
    emu_standin bytes=0
    emu_case "ends at once" --no-output --expect-fail --oxrok-fails "before its first chunk" -- -d $D --samples 4 -O bits
    emu_standin bytes=0 exit=1
    emu_case "fails at once" --no-output --expect-fail --oxrok-fails "before its first chunk" -- -d $D --samples 4 -O bits
    emu_standin bytes=3
    # C closes stdin as soon as it has read the 3 bytes, while the program is on its
    # way out: whether the program saw the close first is the pace's.
    emu_case "a short first chunk" --no-output --expect-fail --oxrok-fails "before its first chunk" --tx-filter '/^stdin closed$/d' -- -d $D --samples 4 -O bits
    emu_standin bytes=0 then=hang hang-max=60000
    # Thirty seconds, oxrok's bound on the first chunk: the one case that waits one
    # out (-version's is a unit test's).
    emu_case "silent at the start" --oracle-bug "$NO_BOUND" --oracle-hangs --timeout 40 --no-output --oxrok-fails "No first chunk" -- -d $D --samples 4 -O bits
    emu_standin piece=5 gap=20
    emu_case "pieces of five" --oracle-bug "$ONE_READ" -- -d $D --samples 4 -O bits
    emu_standin

    # A stream that ends: C's source sees only G_IO_HUP, which its callback ignores,
    # so sigrok-cli spins until its time limit, and without one for ever.
    emu_standin bytes=600 then=exit
    emu_case "cut short" --oracle-bug "$HUP" --oracle-hangs --timeout 4 --stderr --filter "$L" -- -d $D --samples 100000 -O bits
    emu_case "cut short with a time" --digest --filter "$A" -- -d $D --time 500 -O hex
    emu_standin bytes=603 then=exit
    emu_case "cut inside a chunk" --oracle-bug "$HUP" --oracle-hangs --timeout 4 --stderr --filter "$L" -- -d $D --samples 100000 -O bits
    emu_standin bytes=600 then=exit exit=1
    emu_case "fails inside" --oracle-bug "$HUP" --oracle-hangs --timeout 4 --oxrok-fails "ended the stream" --stderr --filter "$L" -- -d $D --samples 100000 -O bits
    emu_standin bytes=600 then=hang
    emu_case "hangs after its data" --oracle-differs "$ORPHAN" --digest --filter "$A" -- -d $D --time 500 -O hex
    emu_standin bytes=600 then=hang eof=ignore
    emu_case "hangs ignoring stdin" --oracle-differs "$ORPHAN" --digest --filter "$A" -- -d $D --time 500 -O hex
    # A wrapper that doesn't exec, as a script that runs wine is: the program it runs
    # hangs, and oxrok's kill takes it with the wrapper.
    W=$EMU_STANDIN_DIR/omegartmcli-wrap
    printf '#!/bin/sh\n"$(dirname "$0")/omegartmcli" "$@"\nexit $?\n' >"$W" && chmod +x "$W"
    export OMEGARTMCLI="$W"
    emu_standin bytes=600 then=hang hang-max=20000
    emu_case "a wrapper's program hangs" --oracle-differs "$ORPHAN" --digest --filter "$A" -- -d $D --time 300 -O hex
    unset OMEGARTMCLI
    # Descriptors the tool was handed beyond the three, as a CI runner hands two:
    # glib's spawn closes them in the program, and oxrok's must too. EMU_WRAP for
    # these cases alone.
    F=$EMU_STANDIN_DIR/with-fds
    printf '#!/bin/sh\nexec "$@" 3</dev/null 4</dev/null\n' >"$F" && chmod +x "$F"
    emu_standin
    EMU_WRAP=$F
    emu_case "descriptors beyond the three" -- -d $D --samples 4 -O bits
    emu_case "descriptors beyond the three at a scan" --keep-header -- -d $D --scan
    EMU_WRAP=""
    # The time limit starts after the first chunk (api.c:375): a program slower to
    # start than the limit still gives its samples.
    emu_standin start=1500 bytes=1800 then=quiet piece=600 gap=50
    emu_case "a slow start and a time limit" --digest --filter "$A" -- -d $D --time 1000 -O hex
    emu_standin

    # SIGINT, as the tools' pace has it: at chunk 100000 of a stream.
    emu_standin mark=100000
    emu_case "interrupt" --oracle-bug "$SIGINT_BUG" --interrupt tcp-model --stderr --no-output -- -d $D --time 600000 -O hex
    emu_standin mark=100000 bytes=601200 then=hang
    emu_case "interrupt a hung program" --oracle-bug "$SIGINT_BUG" --interrupt tcp-model --stderr --filter "$L" -- -d $D --time 600000 -O hex
    # While the program configures its FPGA, before its first chunk: oxrok's stop
    # reaches the wait for it.
    emu_standin mark=0 start=20000
    emu_case "interrupt before the first chunk" --oracle-bug "$SIGINT_BUG" --interrupt tcp-model --timeout 15 --stderr --filter "$L" -- -d $D --time 600000 -O hex
    emu_standin
}
