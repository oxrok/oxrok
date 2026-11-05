# A Rohde&Schwarz SME signal generator (emu/src/models/rohde-schwarz-sme-0x.rs),
# SCPI on the CDC-ACM shell. Sourced by tools/diffcli.sh --emu; see docs/USB.md,
# "Writing a serial model".
EMU_MODEL=rohde-schwarz-sme-0x
EMU_CDC=1
EMU_DRIVER=rohde-schwarz-sme-0x
EMU_GCOV=rohde-schwarz-sme-0x
EMU_GCOV_SHARED="src/serial.c src/serial_libsp.c src/scpi/scpi.c src/scpi/scpi_serial.c"

emu_cases() {
    emu_conns
    # What a failed read prints: a double nobody set, api.c:170, so what the stack
    # holds. On a line that is a bare number, or the --show line of $KEY.
    NEVER_SET='s/^[-+0-9.e]+$/(never set)/'
    FREQ_NEVER_SET='s/^( *output_frequency: ).*$/\1(never set)/'
    POW_NEVER_SET='s/^( *amplitude: ).*$/\1(never set)/'

    # A scan: a bare LF, then *IDN?, on a port nobody set the line of.
    emu_options
    emu_case "scan" --keep-header -- -d $R --scan
    for rm in SME02 SME03E SME03A SME06; do
        emu_options model=$rm
        emu_case "scan $rm" --keep-header -- -d $R --scan
    done
    emu_options model=SMT03
    emu_case "scan a model not in the table" --keep-header --no-output -- -d $R --scan
    emu_options model=sme03
    emu_case "scan in lower case" --keep-header --no-output -- -d $R --scan
    emu_options idn='ROHDE&SCHWARZ,SME03,1,1'
    emu_case "scan another spelling of the maker" --keep-header --no-output -- -d $R --scan
    emu_options idn='Rohde&Schwarz,SME03,1.00'
    emu_case "scan three fields" --keep-header --stderr -- -d $R --scan
    emu_options idn='Rohde&Schwarz,SME03'
    emu_case "scan two fields" --keep-header --no-output -- -d $R --scan
    # IDN Rohde&Schwarz, SME03 ,sn , 2.5 ,extra
    emu_options rawidn=49444e20526f686465265363687761727a2c20534d453033202c736e202c20322e35202c65787472610a
    emu_case "scan with spaces and a prefix" --keep-header -- -d $R --scan
    emu_options rawidn=0a
    emu_case "scan an empty line" --keep-header --no-output -- -d $R --scan
    emu_options rawidn=none
    emu_case "scan silent" --keep-header --no-output -- -d $R --scan
    # The end of a line: only what a read ends with counts.
    emu_options eol=crlf
    emu_case "scan with CR LF" --keep-header -- -d $R --scan
    emu_options eol=lfcr
    emu_case "scan with LF CR" --keep-header -- -d $R --scan
    emu_options chunk=1
    emu_case "scan one byte per transfer" --keep-header -- -d $R --scan
    # The line: no serialcomm sets nothing, so what the tty has is what it gets.
    emu_options uart=115200/8n1
    emu_case "scan at another rate" --keep-header --no-output -- -d $R --scan
    emu_case "serialcomm fits" --keep-header -- -d $R:serialcomm=115200/8n1 --scan
    emu_options
    emu_case "serialcomm does not fit" --keep-header --no-output -- -d $R:serialcomm=115200/8n1 --scan
    emu_case "serialcomm no such thing" --keep-header --no-output -- -d $R:serialcomm=fast --scan
    emu_options stall=lines
    emu_case "control lines stalled" --keep-header -- -d $R --scan
    emu_options
    emu_case "no such node" --keep-header --no-output -- -d rohde-schwarz-sme-0x:conn=/dev/ttyACM97 --scan
    emu_case "no conn" --keep-header --no-output -- -d rohde-schwarz-sme-0x --scan
    emu_case "show without a device" --keep-header --stderr --expect-fail -- -d rohde-schwarz-sme-0x --show

    # scpi_serial.c's table of USB ids that need no conn and bring a line.
    emu_options ids=0aad:0117 uart=115200/8n1
    emu_case "no conn, a known USB serial port" --keep-header -- -d rohde-schwarz-sme-0x --scan
    emu_case "conn, a known USB serial port" --keep-header -- -d $R --scan

    # What --show and --get read.
    emu_options
    emu_case "show" --keep-header -- -d $R --show
    emu_case "get output_frequency" -- -d $R --get output_frequency
    emu_case "get amplitude" -- -d $R --get amplitude
    emu_case "get no such key" --stderr --expect-fail -- -d $R --get offset
    emu_options freq=2500000000.5 pow=-144
    emu_case "get a fraction" -- -d $R --get output_frequency
    emu_case "get the bottom level" -- -d $R --get amplitude
    # A reply's forms; the driver takes all that sr_atod_ascii does.
    emu_options rawfreq=2031303030300a
    emu_case "get a reading with a leading space" -- -d $R --get output_frequency
    emu_options rawfreq=312e303045390a
    emu_case "get a reading with an exponent" -- -d $R --get output_frequency
    emu_options eol=crlf
    emu_case "get with CR LF" -- -d $R --get output_frequency
    emu_options chunk=1
    emu_case "get one byte per transfer" -- -d $R --get output_frequency

    # A read that fails: api.c:176-177 and :180-181 drop the result and hand back
    # a double they never set.
    emu_options rawfreq=none
    emu_case "get silent" --filter "$NEVER_SET" -- -d $R --get output_frequency
    emu_options rawpow=none
    emu_case "get amplitude silent" --filter "$NEVER_SET" -- -d $R --get amplitude
    emu_case "show amplitude silent" --keep-header --filter "$POW_NEVER_SET" -- -d $R --show
    emu_options rawfreq=6162630a
    emu_case "get no number" --filter "$NEVER_SET" -- -d $R --get output_frequency
    emu_options rawpow=2d33302020200a
    emu_case "get a reading with a tail" --filter "$NEVER_SET" -- -d $R --get amplitude
    # sr_atod_ascii's 0 for nothing, and what it lets through.
    emu_options rawpow=0a
    emu_case "get an empty line" -- -d $R --get amplitude
    emu_options rawpow=696e660a
    emu_case "get inf" -- -d $R --get amplitude
    emu_options rawpow=6e616e0a
    emu_case "get nan" -- -d $R --get amplitude
    emu_options rawfreq=3078313030300a
    emu_case "get in hexadecimal" -- -d $R --get output_frequency
    emu_options rawfreq=310a
    emu_case "get a frequency that is no frequency" -- -d $R --get output_frequency
    # A reply no query's own. shift has each answer one query behind, with no clock.
    emu_options shift=1
    emu_case "show one behind" --keep-header --filter "$FREQ_NEVER_SET" -- -d $R --show

    # Capture: nothing to acquire and no limit key.
    emu_options
    emu_case "samples" --keep-header --stderr --expect-fail -- -d $R --samples 1
    emu_case "time" --keep-header --stderr --expect-fail -- -d $R --time 100
    emu_case "continuous" --keep-header --stderr --expect-fail -- -d $R --continuous
    emu_case "lib capture has no enabled channels" --libsigrok --expect-fail --stderr -- -d $R --continuous

    # Settings.
    emu_options
    emu_case "set output_frequency" --no-output -- -d $R -c output_frequency=1000000 --set
    emu_case "set a fraction" --no-output -- -d $R -c output_frequency=5000.5 --set
    emu_case "set rounds to one decimal" --no-output -- -d $R -c amplitude=-10.25 --set
    emu_case "set amplitude" --no-output -- -d $R -c amplitude=-10.5 --set
    emu_case "set both" --no-output -- -d $R -c amplitude=-10.5:output_frequency=2000000 --set
    emu_case "set then get" -- -d $R -c amplitude=-10.5 --get amplitude
    emu_case "set no such key" --stderr --expect-fail -- -d $R -c offset=1 --set
    emu_case "set the bottom frequency" --no-output -- -d $R -c output_frequency=5000 --set
    emu_case "set the top frequency" --no-output -- -d $R -c output_frequency=3000000000 --set
    emu_case "set the top level" --no-output -- -d $R -c amplitude=16 --set
    emu_case "set the bottom level" --no-output -- -d $R -c amplitude=-144 --set
    emu_case "set minus zero" --no-output -- -d $R -c amplitude=-0 --set
    # Out of range: nothing goes out, no word of it, exit 0.
    emu_case "set under the bottom frequency" --no-output -- -d $R -c output_frequency=4999 --set
    emu_case "set over the top frequency" --no-output -- -d $R -c output_frequency=3000000256 --set
    emu_case "set over the top level" --no-output -- -d $R -c amplitude=16.5 --set
    emu_case "set under the bottom level" --no-output -- -d $R -c amplitude=-144.5 --set
    emu_case "set nan" --no-output -- -d $R -c output_frequency=nan --set
    emu_case "set no number" --no-output -- -d $R -c amplitude=abc --set
    # One good, one bad.
    emu_case "set one of two out of range" --no-output -- -d $R -c output_frequency=2000000:amplitude=99 --set
    # What single precision does to a frequency: sigrok-cli's strtof.
    emu_case "set a frequency over 2^24" --no-output -- -d $R -c output_frequency=123456789 --set
    # Each model's top, and the SME06's table entry.
    for rm in SME02:1500000000:1500000256 SME03E:2200000000:2200000256 SME03A:3000000000:3000000256; do
        rn=${rm%%:*}; rt=${rm#*:}; rv=${rt#*:}; rt=${rt%:*}
        emu_options model=$rn
        emu_case "set the top of $rn" --no-output -- -d $R -c output_frequency=$rt --set
        emu_case "set over the top of $rn" --no-output -- -d $R -c output_frequency=$rv --set
    done
    emu_options model=SME06
    emu_case "set the top of the SME06 in its table" --no-output -- -d $R -c output_frequency=1500000000 --set
    emu_case "set the SME06 at 3 GHz" --no-output -- -d $R -c output_frequency=3000000000 --set
    # A device lost.
    emu_options answers=0
    emu_case "unplug in a get" --filter "$NEVER_SET" --unplug model --timeout 5 --tx-filter '/^unplug$/q' -- -d $R --get amplitude
    emu_options
}

# See emu_options in tools/diffcli.sh.
emu_conns() {
    R=rohde-schwarz-sme-0x:conn=$EMU_TTY
}
