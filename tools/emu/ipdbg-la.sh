# The IPDBG logic analyzer core (emu/src/models/ipdbg-la.rs), on a socket alone.
# Sourced by tools/diffcli.sh --emu; see docs/USB.md, "Writing a SCPI model".
EMU_MODEL=ipdbg-la
EMU_TCP=only
EMU_DRIVER=ipdbg-la
EMU_FULL=$EMU_DRIVER
EMU_GCOV=ipdbg-la

# Stock C sends no SR_DF_HEADER: dev_acquisition_start (api.c:240-258) has no
# std_session_send_df_header, and sigrok-cli's datafeed_in drops every packet before
# a header (session.c:285-287), so any capture of stock C prints nothing and exits 0.
# The driver's first commit sent one (a4210e18, 2017-05-16), and "ipdbg-la:
# stop-function pulseview" (b8fa29a1, 2017-09-28) took it out before the merge. The
# core maintainer's branches put it back, and IPDBG's co-sim CI builds sigrok-cli
# against one of them (danselmi/libsigrok ipdbg-la-and-awg). So the captures below
# compare with a C that gets a header from tools/oracle-header.c (emu_header_oracle),
# ahead of the first packet the driver sends, and with no other change: the driver
# and the session are as built. "stock" cases pin what stock does.

emu_cases() {
    emu_header_oracle
    T=ipdbg-la:conn=tcp-raw/127.0.0.1/$EMU_TCP_PORT
    # A capture's output in few lines, and through --digest the hash of all of it.
    A='1~64!d'
    L='s/^oxrok: [a-z]+: oxrok::drivers::ipdbg_la(::protocol)?: /sr: ipdbg-la: /'
    # The words of the system's resolver are its own, in both tools.
    R0='s/(invalid argument).*$/\1./'
    W="$L
s/^(sr: ipdbg-la: Address lookup failed: [^:]+:)[0-9]*([^ ]*): .*/\1PORT\2: WHY/"
    NO_HEADER="dev_acquisition_start (api.c:240-258) sends no header and sigrok-cli drops what comes before one (session.c:285-287), so stock prints no sample and exits 0; oxrok sends one where the maintainer's branch does"
    STRSPLIT="ipdbg_la_split_addr_port reads strs[2] of a list of two (api.c:51), which is a crash on a conn= of one field; oxrok finds no device"
    NO_PORT="C's %s of a missing port prints (null) (protocol.c:133); oxrok says no port"
    UNINIT="buf[8] (protocol.c:479) is read uninitialised with no answer (protocol.c:488-496), so sigrok-cli makes channels from what the stack holds and is still at it after the timeout; oxrok finds no device"
    SHIFT="0x01 << addr_width is an int (protocol.c:505): 31 is 18446744071562067968 samples and 32 is 1 on x86. No core has 31 bits (LogicAnalyserMemory.vhd's 2**ADDR_WIDTH is a VHDL integer); oxrok finds no device"
    GONE="ipdbg_la_tcp_receive asks FIONREAD, which is 0 for ever after a close (protocol.c:203), and ipdbg_la_receive_data returns TRUE with its source kept, so a gone device holds sigrok-cli for ever; oxrok's capture fails"
    WAIT_TRIGGER="sigrok-cli's -w keeps samples from its decoders alone (session.c:402-404), and its output module gets them all, where oxrok's -w holds them back from the output too (src/cli/pipeline.rs)"
    QUIET_LIMIT="a limit over the memory is ignored, and the old one stays (api.c:212); oxrok does the same and warns"
    SIGINT_BUG="sigrok-cli has no SIGINT handler, it dies where oxrok ends the capture"

    emu_case "scan" --keep-header -- -d $T --scan
    emu_case "show" --keep-header -- -d $T --show
    emu_case "get limit" -- -d $T --get limit_samples
    emu_case "get ratio" -- -d $T --get captureratio
    emu_case "get what is not there" --no-output --expect-fail -- -d $T --get samplerate
    emu_case "set limit" -- -d $T -c limit_samples=100 --get limit_samples
    emu_case "set limit over" -- -d $T -c limit_samples=513 --get limit_samples
    emu_case "set limit the most" -- -d $T -c limit_samples=512 --get limit_samples
    emu_case "set ratio" -- -d $T -c captureratio=25 --get captureratio
    emu_case "set ratio over" --stderr --expect-fail --filter "$R0" -- -d $T -c captureratio=101 --get captureratio
    emu_case "set only" --no-output -- -d $T -c captureratio=25 --set
    emu_case "set limit over says so" --stderr --filter "$L" --oracle-differs "$QUIET_LIMIT" -- -d $T -c limit_samples=513 --set

    # conn=: g_strsplit(conn, "/", 3). The first field isn't looked at.
    emu_case "conn any first field" --keep-header -- -d ipdbg-la:conn=whatever/127.0.0.1/$EMU_TCP_PORT --scan
    emu_case "conn empty first field" --keep-header -- -d ipdbg-la:conn=/127.0.0.1/$EMU_TCP_PORT --scan
    emu_case "conn host name" --keep-header -- -d ipdbg-la:conn=tcp-raw/localhost/$EMU_TCP_PORT --scan
    emu_case "conn one field" --no-output --oracle-bug "$STRSPLIT" -- -d ipdbg-la:conn=tcp-raw --scan
    emu_case "conn no port" --stderr --filter "$L" --oracle-differs "$NO_PORT" -- -d ipdbg-la:conn=tcp-raw/127.0.0.1 --scan
    emu_case "conn no conn" --no-output -- -d ipdbg-la --scan
    emu_case "conn more fields" --stderr --filter "$W" -- -d ipdbg-la:conn=tcp-raw/127.0.0.1/$EMU_TCP_PORT/x --scan
    emu_case "conn a closed port" --stderr --filter "$L" -- -d ipdbg-la:conn=tcp-raw/127.0.0.1/1 --scan
    emu_case "conn no such host" --stderr --filter "$W" -- -d ipdbg-la:conn=tcp-raw/256.256.256.256/1 --scan

    # A scan's answers.
    emu_options id=78585858
    emu_case "wrong id" --stderr --filter "$L" -- -d $T --scan
    emu_options id=69646267
    emu_case "a core with run length compression" --stderr --filter "$L" -- -d $T --scan
    emu_options id=4944
    emu_case "short id" --stderr --filter "$L" -- -d $T --scan
    emu_options mute=id
    emu_case "silent at the id" --stderr --filter "$L" -- -d $T --scan
    emu_options hang=id
    emu_case "closes at the id" --stderr --filter "$L" -- -d $T --scan
    emu_options late=id:10
    emu_case "late id" --keep-header -- -d $T --scan
    emu_options late=id:150
    emu_case "an id later than the tries" --stderr --filter "$L" -- -d $T --scan
    emu_options late=sizes:10
    emu_case "late sizes" --keep-header -- -d $T --scan
    emu_options sizes=0800000004
    emu_case "short sizes" --stderr --filter "$L" -- -d $T --scan
    emu_options mute=sizes
    emu_case "silent at the sizes" --oracle-bug "$UNINIT" --oracle-hangs --stderr --filter "$L" --timeout 6 -- -d $T --scan
    emu_options hang=sizes
    emu_case "closes at the sizes" --oracle-bug "$UNINIT" --oracle-hangs --stderr --filter "$L" --timeout 6 -- -d $T --scan

    # Widths.
    for wa in 1:4 7:4 8:4 9:4 15:4 16:1 17:4 32:4 64:3 65:3 16:0 16:8 16:16; do
        emu_options data-width=${wa%:*} addr-width=${wa#*:}
        emu_case "width $wa show" --keep-header -- -d $T --show
        emu_case "width $wa capture" --digest --filter "$A" -- -d $T --samples 8 -O bits
    done
    emu_options data-width=0
    emu_case "no channels scan" --keep-header -- -d $T --scan
    # sigrok-cli's two lines rewritten as oxrok's one, so the golden holds oxrok's.
    C='s/^sr: session: ipdbg-la device \(null\) has no enabled channels\.$/ipdbg-la device has no enabled channels/
/^Failed to start session\.$/d'
    emu_case "no channels capture" --stderr --expect-fail --filter "$C" -- -d $T --samples 8 -O bits
    emu_options addr-width=30
    emu_case "address width 30" --keep-header -- -d $T --show
    for a in 31 32 33 64; do
        emu_options addr-width=$a
        emu_case "address width $a" --oracle-differs "$SHIFT" --expect-fail --stderr --filter "$L" -- -d $T --show
    done
    emu_options

    # Captures.
    emu_case "capture bits" -- -d $T --samples 8 -O bits
    emu_case "capture hex all" --digest --filter "$A" -- -d $T --samples 512 -O hex
    emu_case "capture binary" --keep-header --digest --filter "1~512!d" -- -d $T --samples 512 -O binary
    emu_case "capture csv trigger" --digest -- -d $T --samples 8 -O csv:trigger=true:header=false
    emu_case "capture one sample" -- -d $T --samples 1 -O bits
    emu_case "capture two" -- -d $T --samples 2 -O bits
    emu_case "limit by config" -- -d $T -c limit_samples=8 --samples 8 -O bits
    emu_case "limit past the memory" --digest --filter "$A" -- -d $T --samples 513 -O hex
    emu_case "limit zero" --stderr --filter "$L" --expect-fail -- -d $T --samples 0 -O bits
    emu_case "time" --stderr --filter "$L" --expect-fail -- -d $T --time 10 -O bits
    emu_case "continuous" --stderr --filter "$L" --expect-fail -- -d $T --continuous -O bits
    emu_case "frames" --stderr --filter "$L" --expect-fail -- -d $T --frames 2 -O bits
    for r in 0 1 33 50 99 100; do
        emu_case "ratio $r" --digest -- -d $T -c captureratio=$r --samples 16 -O csv:trigger=true:header=false
        emu_case "ratio $r all" --digest --filter "$A" -- -d $T -c captureratio=$r --samples 512 -O hex
    done
    emu_case "channels subset" -- -d $T -C CH0,CH9 --samples 8 -O bits
    emu_case "a delay to escape" -- -d $T --samples 171 -O hex
    emu_case "a delay to escape ee" -- -d $T --samples 477 -O hex

    # Triggers: the wire says what each is, and where the model finds it.
    for t in CH3=0 CH3=1 CH3=r CH3=f CH3=e CH9=r CH15=f CH0=1,CH2=1,CH4=1,CH6=1 CH1=1,CH2=1,CH3=1,CH5=1,CH6=1,CH7=1 CH1=r,CH2=0; do
        emu_case "trigger $t" --digest -- -d $T -t $t --samples 16 -O csv:trigger=true:header=false
    done
    emu_case "trigger on a disabled channel" --stderr --filter "$L" --expect-fail -- -d $T -C CH0,CH1 -t CH3=r --samples 8 -O bits
    emu_case "trigger a bad match" --stderr --filter "$L" --expect-fail -- -d $T -t CH3=x --samples 8 -O bits
    emu_case "wait trigger" --oracle-differs "$WAIT_TRIGGER" -- -d $T -w -t CH3=r --samples 8 -O bits
    emu_options data-width=32
    emu_case "trigger on a wide core" --digest -- -d $T -t CH25=r --samples 8 -O csv:trigger=true:header=false
    emu_options

    # What a real pace does.
    emu_options first=10
    emu_case "a slow trigger" -- -d $T --samples 8 -O bits
    emu_options addr-width=12 pace=512
    emu_case "real pace" --digest --filter "$A" -- -d $T --samples 4096 -O hex
    emu_case "real pace with a limit" -- -d $T --samples 8 -O bits
    # A dump cut at odd bytes, so pieces end inside samples: the limit's end (byte 16
    # of 8 samples, 18 of 9) and the delay's (byte 6 of 3 samples, 8 of 4) fall inside
    # or at the end of a piece of three.
    emu_options addr-width=4 pace=3
    emu_case "pieces of three, 8 samples" -- -d $T --samples 8 -O bits
    emu_case "pieces of three, 9 samples" -- -d $T --samples 9 -O bits
    emu_options addr-width=4 pace=1
    emu_case "pieces of one byte" -- -d $T -c captureratio=33 --samples 11 -O bits
    emu_options gap=100:50
    emu_case "a pause inside a dump" -- -d $T --samples 8 -O bits
    emu_options extra=3
    emu_case "more than a dump" -- -d $T --samples 8 -O bits
    emu_options

    # Gone or silent.
    emu_options close-at=100
    emu_case "closes inside a dump" --timeout 3 --stderr --filter "$L" --oracle-bug "$GONE" --oracle-hangs --oxrok-fails "closed the connection" -- -d $T --samples 8 -O bits
    emu_options hang=arm
    emu_case "closes at the arming" --timeout 3 --no-output --oracle-bug "$GONE" --oracle-hangs --oxrok-fails "closed the connection" -- -d $T --samples 8 -O bits
    # Five seconds, in oxrok's stop: the wait for what a dump has left.
    emu_options cut=100 mark-at=100
    emu_case "silent inside a dump" --oracle-bug "$SIGINT_BUG" --interrupt tcp-model --timeout 20 --stderr --filter "$L" -- -d $T --samples 8 -O bits
    emu_options never=1 mark-arm=1
    emu_case "never triggers" --oracle-bug "$SIGINT_BUG" --interrupt tcp-model --timeout 8 --no-output -- -d $T --samples 8 -O bits
    emu_options
    # C's end here goes with the clock. tcp_send is a plain send() (protocol.c:164) and
    # nothing ignores SIGPIPE: it failed at once in one session and ran out a 60 s timeout
    # in another, and its stderr left its golden in 2 of 15 runs. So oxrok alone.
    emu_case "unplug at the scan's end" --no-oracle-only --unplug tcp-in=2 --timeout 4 --no-output --oxrok-fails "closed the connection" -- -d $T --samples 8 -O bits
    emu_options

    # Stock C.
    SIGROK_CLI=$EMU_STOCK_CLI
    emu_options
    emu_case "stock capture" --oracle-differs "$NO_HEADER" -- -d $T --samples 8 -O bits
    SIGROK_CLI=$EMU_STOCK_CLI
}
