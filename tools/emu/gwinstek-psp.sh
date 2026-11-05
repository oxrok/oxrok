# A GW Instek PSP-603, PSP-405 or PSP-2010 (emu/src/models/gwinstek-psp.rs) on
# the CDC-ACM shell. Sourced by tools/diffcli.sh --emu; see docs/USB.md,
# "Writing a serial model".
EMU_MODEL=gwinstek-psp
EMU_CDC=1
EMU_DRIVER=gwinstek-psp
EMU_FULL=$EMU_DRIVER
EMU_GCOV=gwinstek-psp
EMU_GCOV_SHARED="src/serial.c src/serial_libsp.c"

# An answer to L with its line end, for replies=.
psp_st() {
    emu_hex "$1\\r\\n"
}

emu_cases() {
    emu_conns
    # oxrok's log prefix, on its side only.
    L='s/^oxrok: (err|warn): oxrok::drivers::gwinstek_psp(::protocol)?: /sr: gwinstek-psp: /'
    # What oxrok adds after the error sigrok-cli names.
    A='s/^(Failed to set device option .[a-z_]+.: invalid argument).*$/\1./'
    NA="s/^(Failed to get .[a-z_]+.: )not supported.*$/\\1not applicable/"
    # hwdriver.c's check_key refuses it ahead of the driver, and logs that.
    Z="$A
/^sr: hwdriver: Cannot set/d"
    NAN_BUG="api.c:254 and :277's two comparisons let a NaN through, and api.c:259, :263 and :269 send SU -2147483648 and 'SV   nan' to a live output; oxrok refuses it"
    ZERO_BUG="api.c:254 and :277 let -0.0 through, and :269 and :280 send it as -0.00, which nothing says the supply parses; oxrok sends 0"
    LOOSE_BUG="sigrok-cli takes what is no value (session.c's opt_to_gvar, strtof) and sends what it makes of it, here a target of 0 V; oxrok refuses it and sends nothing"
    ORDER_BUG="sigrok-cli sets each -c in turn (session.c:700-707), so KOE goes out before the set point that follows; oxrok sends the output on last and off first"
    PARTIAL_BUG="sigrok-cli sets each -c in turn (session.c:700-707), so KOE goes out before a later option is refused; oxrok holds KOE back until the rest was accepted"
    STALE_LIMIT_BUG="api.c:260 compares with the voltage limit of the last L (protocol.c:158-160), and nothing notes the SU it sent at :263: a second target in the run takes the old limit for the device's, so here SV 60.00 goes out under a limit of 5 V, or SU is sent twice; oxrok keeps the limit it sent"
    READBACK_BUG="config_set stores nothing a get reads (api.c:253-290), and a get within 245 ms answers the last L (protocol.c:136-141), so a value just set reads back as the old one; oxrok answers what was set until the next L"
    CHANNEL_BUG="protocol.c:249-272 sends both readings whatever -C says; oxrok still asks, and sends only what is enabled"
    GET_LIMIT_BUG="api.c:39-40 list both limits as readable, and config_get has no case for them (:228); oxrok answers a limit that was set, and with none set says not applicable as C does"
    FIELD_BUG="sscanf stores each field as it goes (protocol.c:158-160), and the failure's only effect is a log line, as protocol.c:217 drops the result: the next sample has the current of a status that failed; oxrok has no sample from a status that failed, and stores none of it"
    FLOOD_BUG="what is unread after a reply keeps the source ready (api.c:338), and protocol.c's callback reads nothing within 245 ms of the last L but counts a sample (:275-278): the tail of a long reply runs a sample limit out at once, with no further L; oxrok keeps to its 50 ms and goes on asking"
    UNPLUG_BUG="protocol.c:217 drops the result, so a device gone gives the last reading until the sample limit, or for ever without one; oxrok ends the capture"
    HANG_BUG="protocol.c:162 reads until a second's silence, which a device that keeps talking never gives: the scan never ends; oxrok gives up after 3 s"
    SIGINT_BUG="sigrok-cli has no SIGINT handler, it dies where oxrok ends the capture"
    # What a device gone leaves: how many failures and samples are the clock's,
    # and how the port's error reads, or which call meets it, the platform's.
    GONE="$L
/Parsing status payload failed/{x;/seen/{x;d};s/.*/seen/;x}
/^sr: gwinstek-psp: Error /d
s/^(sr: gwinstek-psp: Acquisition failed:).*/\\1 the port went./
s/only sent [0-9]+ samples/only sent some samples/"
    # L is asked only when the last was over 245 ms ago, and how many fall in a
    # run is the clock's. So a run of the same question and answer folds to one,
    # in the transactions and, as V and I packets, in the output. Each pair is
    # compared with the one before it, and anything else between them counts.
    FOLD='/^bulk_out 02 4c0d0a -> 3$/!{x;s/.*//;x}
/^bulk_out 02 4c0d0a -> 3$/{N;x;G;/^(.*)\n\1$/d;g}'
    # A device that stops answering asks again and again, in a row.
    ASKS='s/^bulk_out 02 (4c0d0a)+ -> [0-9]+$/bulk_out 02 L, as often as time allows/'
    PAIRS='/^V: /{N;x;G;/^(.*)\n\1$/d;g}'
    # The last pair only: what the device reads once what the setting changed
    # has been asked for, however many samples went before it.
    LAST='/^V: /{N;$!d}
$!d'

    # A scan is L, a wait, and no setting, which setters=0 would flag.
    emu_options setters=0
    emu_case "scan" --keep-header -- -d $K --scan
    emu_case "scan names the model in lower case" --keep-header -- -d gwinstek-psp:conn=$EMU_TTY:force_detect=psp-603 --scan
    emu_case "scan with text after the model" --keep-header -- -d gwinstek-psp:conn=$EMU_TTY:force_detect=PSP-603-X --scan
    emu_case "scan names no model it has" --filter "$L" --stderr -- -d gwinstek-psp:conn=$EMU_TTY:force_detect=PSP-999 --scan
    emu_case "scan names part of a model" --filter "$L" --stderr -- -d gwinstek-psp:conn=$EMU_TTY:force_detect=PSP --scan
    emu_case "scan with no model" --filter "$L" --stderr -- -d gwinstek-psp:conn=$EMU_TTY --scan
    emu_case "scan names an empty model" --expect-fail --stderr -- -d gwinstek-psp:conn=$EMU_TTY:force_detect= --scan
    emu_case "no conn" --keep-header --no-output -- -d gwinstek-psp --scan
    emu_case "no conn and a model" --keep-header --no-output -- -d gwinstek-psp:force_detect=PSP-603 --scan
    emu_case "no such node" --keep-header --no-output -- -d gwinstek-psp:conn=/dev/ttyACM97:force_detect=PSP-603 --scan
    emu_options setters=0 model=405
    emu_case "scan PSP-405" --keep-header -- -d gwinstek-psp:conn=$EMU_TTY:force_detect=PSP-405 --scan
    emu_options setters=0 model=2010
    emu_case "scan PSP-2010" --keep-header -- -d gwinstek-psp:conn=$EMU_TTY:force_detect=PSP-2010 --scan
    emu_options setters=0 chunk=1
    emu_case "scan one byte per transfer" --keep-header -- -d $K --scan
    # The line: 2400/8n1 by default, and nothing passes until it fits.
    emu_options setters=0 uart=9600/8n1
    emu_case "scan at no rate it is at" --keep-header --no-output -- -d $K --scan
    emu_case "serialcomm fits" --keep-header -- -d $K:serialcomm=9600/8n1 --scan
    emu_options setters=0
    emu_case "serialcomm does not fit" --keep-header --no-output -- -d $K:serialcomm=9600/8n1 --scan
    # sigrok-cli opens the port before it reads the parameters: oxrok refuses
    # them first, and the open's lines are all that differs.
    emu_case "serialcomm no such thing" --keep-header --no-output --tx-filter '/^ctl_out 21 22 /d' -- -d $K:serialcomm=fast --scan
    # The line end decides how long a read is from then on.
    emu_options setters=0 urpsp=1
    emu_case "scan URPSP1" --keep-header -- -d $K --scan
    emu_case "samples URPSP1" --filter "$PAIRS" --tx-filter "$FOLD" -- -d $K --samples 8
    # Faults of the line.
    emu_options setters=0 in-stall=1
    emu_case "a stalled read" --keep-header -- -d $K --scan
    emu_options setters=0 stall=line
    emu_case "line coding stalled" --keep-header --no-output -- -d $K --scan
    emu_options setters=0 stall=lines
    emu_case "control lines stalled" --keep-header -- -d $K --scan

    # Answers gone wrong in the scan.
    emu_options setters=0 answers=0
    emu_case "scan no answer" --filter "$L" --stderr --timeout 8 -- -d $K --scan
    emu_options setters=0 replies=$(emu_hex 'V12.00A0.000W000.0U60I1.00P200F000100')
    emu_case "scan no line end" --filter "$L" --stderr --timeout 8 -- -d $K --scan
    emu_options setters=0 replies=$(psp_st 'V12.00A0.000W000.0U60I1.00P200F00010')
    emu_case "scan one short" --filter "$L" --stderr --timeout 8 -- -d $K --scan
    emu_options setters=0 replies=$(psp_st 'V12.00A0.000W000.0U60I1.00P200F0001000')
    emu_case "scan one long" --keep-header -- -d $K --scan
    emu_options setters=0 replies=$(psp_st 'V12.00A0.000W000.0U60I1.00P200F00010000')
    emu_case "scan two long" --filter "$L" --stderr --timeout 8 -- -d $K --scan
    emu_options setters=0 replies=$(psp_st 'xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx')
    emu_case "scan garbage" --filter "$L" --stderr --timeout 8 -- -d $K --scan
    # The manual: a lower-case letter while the panel sets that limit. The
    # format has U and I in it, and stops ahead of P.
    emu_options setters=0 limit-mode=u
    emu_case "scan the panel sets the voltage limit" --filter "$L" --stderr --timeout 8 -- -d $K --scan
    emu_options setters=0 limit-mode=i
    emu_case "scan the panel sets the current limit" --filter "$L" --stderr --timeout 8 -- -d $K --scan
    emu_options setters=0 limit-mode=p
    emu_case "scan the panel sets the power limit" --keep-header -- -d $K --scan
    # What %f and %d take, and that the frame need not have the manual's widths. A
    # get may be the scan's L or one of its own: the frame for either, and it folds.
    F=$(psp_st 'V1.5e1A.5W-1.0U+7I 2 P200F00010000000')
    emu_options setters=0 replies=$F,$F
    emu_case "scan as sscanf takes it" --tx-filter "$FOLD" -- -d $K --get voltage_target
    emu_case "scan as sscanf takes it, a limit" --tx-filter "$FOLD" -- -d $K --get current_limit
    F=$(psp_st 'VnanA0.000W000.0U60I1.00P200F00000000')
    emu_options setters=0 replies=$F,$F
    emu_case "scan nan" --tx-filter "$FOLD" -- -d $K --get voltage_target
    F=$(psp_st 'V-infA0.000W000.0U60I1.00P200F0000000')
    emu_options setters=0 replies=$F,$F
    emu_case "scan inf" --tx-filter "$FOLD" -- -d $K --get voltage_target
    F=$(psp_st 'V12.00A0.000W000.0U60I0x10P200F000000')
    emu_options setters=0 replies=$F,$F
    emu_case "scan hexadecimal takes P for its exponent" --tx-filter "$FOLD" -- -d $K --get current_limit
    emu_options setters=0 replies=$(psp_st 'V0x10A0.000W000.0U60I1.00P200F0000000')
    emu_case "scan hexadecimal takes the A" --filter "$L" --stderr --timeout 8 -- -d $K --scan
    emu_options setters=0 replies=$(psp_st 'V1eA0.000W000.0U60I1.00P200F000000000')
    emu_case "scan an exponent with no digits" --filter "$L" --stderr --timeout 8 -- -d $K --scan
    emu_options setters=0 replies=$(psp_st 'V12.00A0.000W000.0U99999999999I1.00P2')
    emu_case "scan a limit too wide for an int" --keep-header -- -d $K --scan
    # A NUL ends the text sscanf sees, and the flags are read by position.
    F=$(psp_st 'V12.00A0.000W000.0U60I1.00\000\000\000\000F100100')
    emu_options setters=0 replies=$F,$F
    emu_case "scan a NUL behind the numbers" --tx-filter "$FOLD" -- -d $K --get enabled

    # --show and --get: asked once by the scan, and again only after 245 ms.
    emu_options setters=0
    emu_case "show" --keep-header --tx-filter "$FOLD" -- -d $K --show
    emu_options setters=0 model=405
    emu_case "show PSP-405" --keep-header --tx-filter "$FOLD" -- -d gwinstek-psp:conn=$EMU_TTY:force_detect=PSP-405 --show
    emu_options setters=0 model=2010
    emu_case "show PSP-2010" --keep-header --tx-filter "$FOLD" -- -d gwinstek-psp:conn=$EMU_TTY:force_detect=PSP-2010 --show
    # A wrong model says so nowhere: the driver cannot ask.
    emu_options setters=0 model=405
    emu_case "show says what it was told" --keep-header --tx-filter "$FOLD" -- -d $K --show
    emu_options setters=0 output=1 load=10 v=5.00 i=2.00 u=6
    for key in conn voltage voltage_target current current_limit enabled otp_active; do
        emu_case "get $key" --tx-filter "$FOLD" -- -d $K --get $key
    done
    emu_case "get limit_samples" --expect-fail --stderr --filter "$NA" --tx-filter "$FOLD" -- -d $K --get limit_samples
    emu_case "get limit_time" --expect-fail --stderr --filter "$NA" --tx-filter "$FOLD" -- -d $K --get limit_time
    emu_case "get limit_samples that was set" --oracle-bug "$GET_LIMIT_BUG" --tx-filter "$FOLD" -- -d $K -c limit_samples=5 --get limit_samples
    emu_case "get limit_time that was set" --oracle-bug "$GET_LIMIT_BUG" --tx-filter "$FOLD" -- -d $K -c limit_time=5 --get limit_time
    emu_options setters=0 v=5.00 i=2.00
    emu_case "get voltage with the output off" --tx-filter "$FOLD" -- -d $K --get voltage
    emu_case "get voltage_target with the output off" --tx-filter "$FOLD" -- -d $K --get voltage_target
    emu_case "get enabled off" --tx-filter "$FOLD" -- -d $K --get enabled
    # Constant current: the voltage is not the target, and the driver sees it.
    emu_options setters=0 output=1 load=1 v=12.00 i=2.00
    emu_case "get voltage_target in CC" --tx-filter "$FOLD" -- -d $K --get voltage_target
    emu_case "scan in CC warns" --filter "$L" --stderr -- -d $K --scan
    emu_case "get voltage in CC" --tx-filter "$FOLD" -- -d $K --get voltage
    # The comparison is single precision, 0.01 in double: 1.99 from 2.00 is
    # under 0.01 here, and 1.98 is over.
    emu_options setters=0 replies=$(psp_st 'V12.00A1.990W000.0U60I2.00P200F100100')
    emu_case "scan 10 mA from the limit" --filter "$L" --stderr -- -d $K --scan
    emu_options setters=0 replies=$(psp_st 'V12.00A1.980W000.0U60I2.00P200F100100')
    emu_case "scan 20 mA from the limit" --stderr --no-output -- -d $K --scan
    emu_options setters=0 otp=1
    emu_case "get over temperature" --tx-filter "$FOLD" -- -d $K --get otp_active

    # A setting: SU if the limit is not already right, then SV.
    emu_options
    emu_case "set enabled on" --no-output -- -d $K -c enabled=on --set
    emu_case "set enabled off" --no-output -- -d $K -c enabled=off --set
    emu_case "set voltage" --no-output -- -d $K -c voltage_target=5 --set
    emu_case "set voltage with a fraction" --no-output -- -d $K -c voltage_target=5.5 --set
    emu_case "set voltage in single precision" --no-output -- -d $K -c voltage_target=3.3 --set
    emu_case "set voltage max" --no-output -- -d $K -c voltage_target=60 --set
    emu_case "set voltage min" --no-output -- -d $K -c voltage_target=0 --set
    emu_case "set voltage rounds down" --no-output -- -d $K -c voltage_target=1.234 --set
    emu_case "set voltage rounds up" --no-output -- -d $K -c voltage_target=1.236 --set
    emu_case "set voltage a tie" --no-output -- -d $K -c voltage_target=0.125 --set
    emu_case "set voltage another tie" --no-output -- -d $K -c voltage_target=0.375 --set
    emu_case "set voltage under a step" --no-output -- -d $K -c voltage_target=0.001 --set
    emu_case "set current" --no-output -- -d $K -c current_limit=1.5 --set
    emu_case "set current max" --no-output -- -d $K -c current_limit=3.5 --set
    emu_case "set current min" --no-output -- -d $K -c current_limit=0 --set
    emu_case "set current a tie" --no-output -- -d $K -c current_limit=0.125 --set
    emu_case "set voltage too high" --filter "$A" --stderr --expect-fail -- -d $K -c voltage_target=60.01 --set
    emu_case "set voltage negative" --filter "$A" --stderr --expect-fail -- -d $K -c voltage_target=-1 --set
    emu_case "set current too high" --filter "$A" --stderr --expect-fail -- -d $K -c current_limit=3.51 --set
    emu_case "set current negative" --filter "$A" --stderr --expect-fail -- -d $K -c current_limit=-0.1 --set
    emu_case "set voltage nan" --oracle-differs "$NAN_BUG" --expect-fail --stderr --filter "$A" -- -d $K -c voltage_target=nan --set
    emu_case "set current nan" --oracle-differs "$NAN_BUG" --expect-fail --stderr --filter "$A" -- -d $K -c current_limit=nan --set
    emu_case "set voltage -0" --oracle-differs "$ZERO_BUG" --no-output -- -d $K -c voltage_target=-0 --set
    emu_case "set current -0" --oracle-differs "$ZERO_BUG" --no-output -- -d $K -c current_limit=-0 --set
    emu_case "set voltage no number" --oracle-differs "$LOOSE_BUG" --expect-fail --stderr --filter "$A" -- -d $K -c voltage_target=abc --set
    emu_case "set conn" --stderr --expect-fail -- -d $K -c conn=x --set
    emu_case "set voltage reading" --stderr --expect-fail -- -d $K -c voltage=1 --set
    emu_case "set over temperature" --stderr --expect-fail -- -d $K -c otp_active=1 --set
    # The limit is sent only when it differs from what L said.
    emu_options u=5
    emu_case "set voltage at the limit" --no-output -- -d $K -c voltage_target=5 --set
    emu_case "set voltage under the limit" --no-output -- -d $K -c voltage_target=4.5 --set
    emu_case "set voltage over the limit" --no-output -- -d $K -c voltage_target=5.01 --set
    emu_case "set voltage a whole number under" --no-output -- -d $K -c voltage_target=4 --set
    # Each model's ranges.
    emu_options model=405
    emu_case "set PSP-405 max" --no-output -- -d gwinstek-psp:conn=$EMU_TTY:force_detect=PSP-405 -c voltage_target=40 -c current_limit=5 --set
    emu_case "set PSP-405 voltage too high" --filter "$A" --stderr --expect-fail -- -d gwinstek-psp:conn=$EMU_TTY:force_detect=PSP-405 -c voltage_target=40.01 --set
    emu_case "set PSP-405 current too high" --filter "$A" --stderr --expect-fail -- -d gwinstek-psp:conn=$EMU_TTY:force_detect=PSP-405 -c current_limit=5.01 --set
    emu_options model=2010
    emu_case "set PSP-2010 max" --no-output -- -d gwinstek-psp:conn=$EMU_TTY:force_detect=PSP-2010 -c voltage_target=20 -c current_limit=10 --set
    emu_case "set PSP-2010 voltage too high" --filter "$A" --stderr --expect-fail -- -d gwinstek-psp:conn=$EMU_TTY:force_detect=PSP-2010 -c voltage_target=20.01 --set
    emu_case "set PSP-2010 current too high" --filter "$A" --stderr --expect-fail -- -d gwinstek-psp:conn=$EMU_TTY:force_detect=PSP-2010 -c current_limit=10.01 --set
    # A range the unit does not have: only the force_detect is asked, and the
    # panel holds what is above it, by api.c's comment.
    emu_options model=405
    emu_case "set above what the unit takes" --no-output -- -d $K -c voltage_target=50 --set
    # Several in one run.
    emu_options
    emu_case "set several in hash order" --no-output -- -d $K -c current_limit=0.5:voltage_target=3.3 --set
    emu_case "set the output and a voltage" --no-output -- -d $K -c voltage_target=3.3 -c enabled=on --set
    emu_case "output goes on after the set point" --oracle-differs "$ORDER_BUG" --no-output -- -d $K -c enabled=on -c voltage_target=3.3 --set
    emu_case "output goes off first" --oracle-differs "$ORDER_BUG" --no-output -- -d $K -c voltage_target=3.3 -c enabled=off --set
    emu_case "output with a voltage too high" --oracle-bug "$PARTIAL_BUG" --expect-fail --stderr --filter "$A" -- -d $K -c enabled=on -c voltage_target=99 --set
    emu_case "output with an unknown option" --oracle-bug "$PARTIAL_BUG" --expect-fail --stderr -- -d $K -c enabled=on:volage_target=5 --set
    emu_case "a set point goes out ahead of a refused one" --oracle-bug "$PARTIAL_BUG" --expect-fail --stderr --filter "$A" -- -d $K -c current_limit=1:voltage_target=99 -c enabled=on --set
    emu_case "two targets in one run" --oracle-differs "$STALE_LIMIT_BUG" --no-output -- -d $K -c voltage_target=5 -c voltage_target=60 --set
    emu_case "the same target twice" --oracle-differs "$STALE_LIMIT_BUG" --no-output -- -d $K -c voltage_target=5 -c voltage_target=5 --set
    # Read back straight after. Inside the 245 ms C answers what the last L said, and
    # a slower box asks again after the settings: what the L after the first is the clock's.
    # The harness joins writes in a row, so that L may be on the end of a setting's
    # line, and the count then differs.
    AGAIN='1,/^bulk_in /!{s/^(bulk_out 02 [0-9a-f]*0d0a)4c0d0a -> [0-9]+$/\1/;s/^(bulk_out 02 [0-9a-f]+) -> [0-9]+$/\1/;/^bulk_out 02 4c0d0a$/d;/^bulk_in /d;}'
    emu_case "set then get voltage" --oracle-differs "$READBACK_BUG" --tx-filter "$AGAIN" -- -d $K -c voltage_target=5 --get voltage_target
    emu_case "set then get current" --oracle-differs "$READBACK_BUG" --tx-filter "$AGAIN" -- -d $K -c current_limit=0.5 --get current_limit
    emu_case "set then get enabled" --oracle-differs "$READBACK_BUG" --tx-filter "$AGAIN" -- -d $K -c enabled=on --get enabled

    # A capture: a sample every 50 ms, an L at most every 245, so the count of
    # L is the clock's and the count of samples the limit's.
    emu_options setters=0 output=1 load=10 v=5.00 i=2.00
    emu_case "samples 1" --tx-filter "$FOLD" -- -d $K --samples 1
    emu_case "samples 3" --tx-filter "$FOLD" -- -d $K --samples 3
    emu_case "samples 8" --tx-filter "$FOLD" -- -d $K --samples 8
    emu_case "channel V only" --oracle-differs "$CHANNEL_BUG" --tx-filter "$FOLD" -- -d $K -C V --samples 2
    emu_case "channel I only" --oracle-differs "$CHANNEL_BUG" --tx-filter "$FOLD" -- -d $K -C I --samples 2
    emu_case "time" --filter '3,$d' --tx-filter "$FOLD" --timeout 5 -- -d $K --time 300ms
    emu_case "config limit_samples 0" --stderr --expect-fail --filter "$Z" -- -d $K -c limit_samples=0 --samples 1
    emu_case "frames refused" --stderr --expect-fail -- -d $K --frames 1
    emu_options setters=0 output=1 load=1 v=12.00 i=2.00
    emu_case "samples in CC" --tx-filter "$FOLD" -- -d $K --samples 2
    # With the output off the voltage reads 0, and the target comes as a meta.
    emu_options setters=0 v=7.50
    emu_case "samples output off" --keep-header --tx-filter "$FOLD" -- -d $K --samples 2
    emu_options setters=0 v=0.00
    emu_case "samples output off at 0 V" --keep-header --tx-filter "$FOLD" -- -d $K --samples 2
    emu_options setters=0 otp=1 output=1
    emu_case "samples over temperature" --keep-header --tx-filter "$FOLD" -- -d $K --samples 2
    emu_options setters=0 output=1
    emu_case "samples at another rate" --filter "$L" --expect-fail --stderr --timeout 8 -- -d $K:serialcomm=9600/8n1 --samples 1
    emu_options setters=0 output=1 uart=9600/8n1
    emu_case "samples at the rate it is at" --tx-filter "$FOLD" -- -d $K:serialcomm=9600/8n1 --samples 2

    # Settings ahead of a capture, and what the device reads once it is asked.
    emu_options load=10
    emu_case "capture with the output on" --filter "$LAST" --tx-filter "$FOLD" -- -d $K -c enabled=on --samples 16
    emu_case "capture with setpoints" --filter "$LAST" --tx-filter "$FOLD" -- -d $K -c current_limit=1:voltage_target=5:enabled=on --samples 16
    emu_case "capture: the output on after the set point" --oracle-differs "$ORDER_BUG" --filter "$LAST" --tx-filter "$FOLD" -- -d $K -c enabled=on -c voltage_target=4 --samples 16
    emu_case "capture after a set" --filter "$LAST" --tx-filter "$FOLD" -- -d $K -c voltage_target=5 --samples 16

    # Replies in turn. The first goes to the scan and the second to the first
    # poll: the same, so that a slow box's first sample is the same either way.
    OFF_12=$(psp_st 'V12.00A0.000W000.0U60I1.00P200F000100')
    OFF_7=$(psp_st 'V07.00A0.000W000.0U60I1.00P200F000100')
    ON_12=$(psp_st 'V12.00A0.500W006.0U60I1.00P200F100100')
    ON_11=$(psp_st 'V11.00A0.750W008.2U60I1.00P200F100100')
    ON_HOT=$(psp_st 'V11.00A0.750W008.2U60I1.00P200F110100')
    emu_options setters=0 output=1 v=11.00 replies=$ON_12,$ON_12,$ON_11,$ON_12,$ON_11
    emu_case "readings in turn" --keep-header --filter "$PAIRS" --tx-filter "$FOLD" -- -d $K --samples 40
    # Metas: what changed, ahead of the sample that shows it.
    emu_options setters=0 output=1 v=11.00 replies=$ON_12,$ON_12,$ON_HOT,$ON_11
    emu_case "over temperature comes and goes" --keep-header --filter "$PAIRS" --tx-filter "$FOLD" -- -d $K --samples 30
    emu_options setters=0 output=1 replies=$OFF_12,$OFF_12,$ON_12
    emu_case "the panel turns the output on" --keep-header --filter "$PAIRS" --tx-filter "$FOLD" -- -d $K --samples 20
    emu_options setters=0 replies=$ON_12,$ON_12,$OFF_12
    emu_case "the panel turns the output off" --keep-header --filter "$PAIRS" --tx-filter "$FOLD" -- -d $K --samples 20
    # Trusted only a second after the last it said.
    emu_options setters=0 v=7.00 replies=$OFF_12,$OFF_12,$OFF_7
    emu_case "the panel changes the target" --keep-header --filter "$PAIRS" --tx-filter "$FOLD" -- -d $K --samples 40
    # A target set and the device agreeing: no meta. Not agreeing: a meta.
    emu_options
    emu_case "a target set says nothing" --keep-header --filter "$PAIRS" --tx-filter "$FOLD" -- -d $K -c voltage_target=7 --samples 40
    emu_options replies=$OFF_12,$OFF_12,$OFF_12,$OFF_12,$OFF_12,$OFF_12,$OFF_12,$OFF_12
    emu_case "a target set the device does not take" --keep-header --filter "$PAIRS" --tx-filter "$FOLD" -- -d $K -c voltage_target=5 --samples 60
    # A frame out of place: stale in C, and resynced by a second's silence.
    emu_options setters=0 replies=$ON_12,$ON_12,$(psp_st 'garbage in place of the status of it!'),$ON_11
    emu_case "a garbage reply" --keep-header --filter "$PAIRS" --tx-filter "$FOLD" --timeout 15 -- -d $K --samples 30
    # The fields ahead of the fault have been stored by then.
    emu_options setters=0 replies=$ON_12,$ON_12,$(psp_st 'V22.00A9.500Wxxx.xU60I1.00P200F000100'),$ON_11
    emu_case "a reply with a bad field" --oracle-differs "$FIELD_BUG" --keep-header --filter "$PAIRS" --tx-filter "$FOLD" --timeout 15 -- -d $K --samples 30
    emu_options setters=0 replies=$ON_12,$ON_12,$(emu_hex 'V11.00'),$ON_11
    emu_case "a reply cut short" --keep-header --filter "$PAIRS" --tx-filter "$FOLD" --timeout 15 -- -d $K --samples 30
    emu_options setters=0 replies=$ON_12,$ON_12,$(psp_st 'V12.00A0.500W006.0U60I1.00P200F100100xx'),$ON_11,$ON_11
    emu_case "a reply too long" --oracle-differs "$FLOOD_BUG" --keep-header --filter "$PAIRS" --tx-filter "$FOLD" --timeout 15 -- -d $K --samples 30
    emu_options setters=0 output=1 limit-mode=u
    emu_case "the panel sets a limit from the start" --filter "$L" --expect-fail --stderr --timeout 8 -- -d $K --samples 1
    # Silent from some L on: each L then takes 1 s and the drain 1 s more. The
    # scan takes one answer and the capture one, so a first sample comes however
    # long the capture took to start, with or without the scan's L still good for
    # it. Those two good for 245 ms each are at most 10 samples, 20 readings, and
    # the pipeline counts readings against the limit, so its warning always comes at
    # 24. C's clock runs from before its 2 s of waiting, so from the silence on each of
    # its samples takes those 2 s: oxrok's line is pinned alone, and C's repeats are in
    # "unplug in a capture says so", which takes no such wait.
    emu_options setters=0 output=1 answers=2
    emu_case "silent from the capture on" --filter "$PAIRS" --tx-filter "$ASKS" --timeout 25 -- -d $K --samples 12
    emu_case "silent from the capture on says so" --no-oracle-only --stderr --filter "$GONE" --tx-filter "$ASKS" --timeout 15 -- -d $K --samples 24
    emu_case "silent with a time limit" --filter "3,\$d" --tx-filter "$ASKS" --timeout 15 -- -d $K --time 1500ms
    # A device that never stops talking, which the drain waits out.
    emu_options setters=0 babble=1
    emu_case "scan a device that keeps talking" --oracle-bug "$HANG_BUG" --oracle-hangs --stderr --filter "$L" --timeout 6 --tx-filter '/^bulk_in /d' -- -d $K --scan
    # An unplug in a poll's wait.
    emu_options setters=0 output=1 answers=2
    emu_case "unplug in a capture" --filter "$PAIRS" --unplug model --timeout 15 --tx-filter '/^unplug$/q' -- -d $K --samples 24
    emu_case "unplug in a capture says so" --oracle-differs "$UNPLUG_BUG" --stderr --filter "$GONE" --unplug model --timeout 15 --tx-filter '/^unplug$/q' -- -d $K --samples 24
    # Long enough for C's next L to meet the port gone, which is its write's error.
    emu_case "unplug in a long capture" --filter "$PAIRS" --unplug model --timeout 15 --tx-filter '/^unplug$/q' -- -d $K --samples 30
    # A quiet point: the L goes unanswered, and the signal lands in its wait.
    emu_options setters=0 output=1 answers=2
    emu_case "interrupt" --oracle-bug "$SIGINT_BUG" --filter "$PAIRS" --interrupt model --timeout 15 --tx-filter '/^interrupt$/q' -- -d $K --continuous
    # csv holds a row until each enabled channel has reported: one row a sample.
    emu_options setters=0 output=1 load=10 v=5.00 i=2.00
    emu_case "csv" --tx-filter "$FOLD" -- -d $K --samples 2 -O csv:header=false:time=false
    emu_options
}

# See emu_options in tools/diffcli.sh.
emu_conns() {
    K=gwinstek-psp:conn=$EMU_TTY:force_detect=PSP-603
}
