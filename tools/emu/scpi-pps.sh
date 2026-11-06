# Bench supplies (emu/src/models/scpi-pps.rs), scpi-pps and hpib-pps, on USBTMC and TCP.
# Sourced by tools/diffcli.sh --emu; see docs/USB.md, "Writing a SCPI model".
EMU_MODEL=scpi-pps
EMU_TMC=1
EMU_TCP=1
EMU_DRIVER="scpi-pps hpib-pps"
EMU_FULL=$EMU_DRIVER
EMU_GCOV=scpi-pps
EMU_GCOV_SHARED="src/scpi/scpi.c src/scpi/scpi_usbtmc_libusb.c src/scpi/scpi_tcp.c src/tcp.c"

emu_cases() {
    U=scpi-pps
    T=scpi-pps:conn=tcp-raw/127.0.0.1/$EMU_TCP_PORT
    H=hpib-pps:conn=tcp-raw/127.0.0.1/$EMU_TCP_PORT

    SIGINT_BUG="sigrok-cli has no SIGINT handler, so an interrupt kills it with its capture unfinished and the instrument left remote; oxrok stops and closes"
    ORDER_BUG="sigrok-cli sets -c's options in hash table order (session.c:700-701), here :OUTP ON before the set points, on a live output; oxrok sets enabled=on last"
    NEVER_ENDS="scpi_pps_receive_data returns on a failed read ahead of its limit check (protocol.c:97-98) and its source stays, so on a silent instrument it asks for ever, and not even a time limit ends it; oxrok ends at the limit with a warning"
    TCP_BLOCKS="scpi_tcp_raw_read_data has sr_tcp_read_bytes block (scpi_tcp.c), and its recv() has no timeout, so a silent instrument holds sigrok-cli for ever; oxrok's read has the timeout"
    TCP_EOF="sr_tcp_read_bytes returns the 0 of a closed connection as a read of nothing (tcp.c), and the driver asks again, so the capture dies of SIGPIPE at the next write (exit 141); oxrok says the instrument closed the connection"
    GONE="scpi_pps_receive_data returns on a failed read and its source stays (protocol.c:97-98), so with the device gone it tries again for ever; oxrok's capture fails"
    QUESTION="profiles.c:82-83 has :CURR:PROT:STAT ON? and OFF?, a boolean with a question mark, which the instrument takes as an error, so C's ocp_enabled does nothing with exit 0; oxrok sends ON and OFF"
    EMPTY="sr_atod_ascii takes an empty answer for 0 (strutil.c:317-319, endptr at the NUL): C reads 0 V where the instrument said nothing; oxrok has no reading"
    HEX="g_ascii_strtod reads 0x10 as 16 and nan as a number, so C takes them for readings; a SCPI number has neither form and oxrok has no reading"
    STATUS_EMPTY="sr_atoi takes an empty answer for 0 (strutil.c), so C says UR, unregulated, of an answer with nothing in it; oxrok says it's no status"
    STATUS_JUNK="sr_atoi's failure is dropped, and api.c:542 goes on with reg never set, so C exits 0 with a regulation from the stack, which the filter masks; oxrok says it's no status"
    STACK="s/^'(CV|CC|CC-|UR)'\$/'from the stack'/"
    NO_VALUE="sr_scpi_cmd_resp's failure is dropped and api.c:583 reads the variant it never set: sigrok-cli dies of SIGSEGV; oxrok fails the get"
    NAN_SENT="config_set has no check and %.6f of a NaN makes :SOUR:VOLT nan, which goes to a live output; oxrok refuses it"
    OUT_OF_RANGE="config_set has no check, and neither tool can see a refusal, as a SCPI setting has no answer: both send it and exit 0, and oxrok warns of a value outside what the output lists"

    # A DP832: three outputs, a table with everything, and a beeper.
    emu_case "scan" --keep-header -- -d $U --scan
    emu_case "scan tcp" --keep-header -- -d $T --scan
    emu_case "show" --keep-header -- -d $U --show
    emu_case "show tcp" --keep-header -- -d $T --show
    emu_case "show group" -- -d $T -g 2 --show
    emu_case "get voltage" -- -d $T -g 1 --get voltage
    emu_case "get voltage target" -- -d $T -g 2 --get voltage_target
    emu_case "get current" -- -d $T -g 1 --get current
    emu_case "get current limit" -- -d $T -g 3 --get current_limit
    emu_case "get enabled" -- -d $T -g 1 --get enabled
    emu_case "get regulation" -- -d $T -g 1 --get regulation
    emu_case "get ovp" -- -d $T -g 1 --get ovp_enabled
    emu_case "get ovp active" -- -d $T -g 1 --get ovp_active
    emu_case "get ovp threshold" -- -d $T -g 1 --get ovp_threshold
    emu_case "get ocp" -- -d $T -g 1 --get ocp_enabled
    emu_case "get ocp active" -- -d $T -g 1 --get ocp_active
    emu_case "get ocp threshold" -- -d $T -g 1 --get ocp_threshold
    emu_case "get otp" -- -d $T --get otp
        emu_case "get limit samples" -- -d $T --get limit_samples
    emu_case "get limit time" -- -d $T --get limit_time
    emu_case "set voltage" --no-output -- -d $T -g 2 --config voltage_target=3.3 --set
    emu_case "set current" --no-output -- -d $T -g 3 --config current_limit=0.5 --set
    emu_case "set enabled" --no-output -- -d $T -g 1 --config enabled=on --set
    emu_case "set disabled" --no-output -- -d $T -g 1 --config enabled=off --set
    emu_case "set ovp on" --no-output -- -d $T -g 1 --config ovp_enabled=on --set
    emu_case "set ovp off" --no-output -- -d $T -g 1 --config ovp_enabled=off --set
    emu_case "set ovp threshold" --no-output -- -d $T -g 1 --config ovp_threshold=31.5 --set
    emu_case "set ocp on" --no-output -- -d $T -g 1 --config ocp_enabled=on --set
    emu_case "set ocp off" --no-output -- -d $T -g 1 --config ocp_enabled=off --set
    emu_case "set ocp threshold" --no-output -- -d $T -g 1 --config ocp_threshold=2.5 --set
    emu_case "set otp on" --no-output -- -d $T --config otp=on --set
    emu_case "set otp off" --no-output -- -d $T --config otp=off --set
    emu_case "set usbtmc" --no-output -- -d $U -g 2 --config voltage_target=3.3 --set
    emu_case "set points then on" --no-output --oracle-differs "$ORDER_BUG" -- -d $T -g 2 --config enabled=on:voltage_target=3.3:current_limit=0.5 --set
    # The output comes on after its set points, and a capture reads what they did.
    emu_case "capture" --record -- -d $U --samples 2
    emu_case "capture tcp" -- -d $T --samples 2
    emu_case "capture analog" -- -d $T --samples 1 -O analog
    emu_case "capture one channel" -- -d $T -C I2 --samples 3
    emu_case "capture two outputs" -- -d $T -C V1,V3 --samples 2
    emu_case "capture over a set" --oracle-differs "$ORDER_BUG" -- -d $T -g 1 -c voltage_target=5:current_limit=0.5:enabled=on --samples 2
    emu_options on=1
    emu_case "capture on" -- -d $T --samples 2
    emu_options on=1 load=1
    emu_case "capture in cc" -- -d $T -C V1,I1,P1 --samples 2
    emu_options beeper=0
    emu_case "no beeper" -- -d $T -C V1 --samples 1
    emu_options

    # Each family: a scan and show, a group's show, a reading and a set.
    for m in N5763A N5767A; do
        emu_options model=$m on=1
        emu_case "$m show" --keep-header -- -d $T --show
        emu_case "$m group" -- -d $T -g 1 --show
        emu_case "$m capture" -- -d $T --samples 2
        emu_case "$m set voltage" --no-output -- -d $T -g 1 --config voltage_target=1.5 --set
    done
    emu_case "n5700 ocp on" --no-output --oracle-differs "$QUESTION" -- -d $T -g 1 --config ocp_enabled=on --set
    emu_case "n5700 ocp off" --no-output --oracle-differs "$QUESTION" -- -d $T -g 1 --config ocp_enabled=off --set
    emu_options model=9130 on=1
    emu_case "bk show" --keep-header -- -d $T --show
    emu_case "bk group" -- -d $T -g 3 --show
    emu_case "bk capture" -- -d $T -C P1,V3 --samples 2
    emu_options model=61604 on=1
    emu_case "61604 show" --keep-header -- -d $T --show
    emu_case "61604 group" -- -d $T -g 1 --show
    emu_case "61604 capture" -- -d $T --samples 2
    emu_case "61604 frequency" -- -d $T -g 1 --get output_frequency
    emu_case "61604 frequency target" -- -d $T -g 1 --get output_frequency_target
    emu_case "61604 set frequency" --no-output -- -d $T -g 1 --config output_frequency_target=60.25 --set
    emu_case "61604 set voltage" --no-output -- -d $T -g 1 --config voltage_target=115.04 --set
    emu_options model=DP711 on=1
    emu_case "dp711 group" -- -d $T -g 1 --show
    emu_case "dp711 capture" -- -d $T -C V1 --samples 2
    emu_options model=DP821A on=1
    emu_case "dp821a group" -- -d $T -g 2 --show
    emu_case "dp821a capture" -- -d $T -C I1,I2 --samples 2
    emu_options model=DP831A
    emu_case "dp831a group" -- -d $T -g 3 --show
    emu_options model=P4305 on=1
    emu_case "p4305 group" -- -d $T -g 1 --show
    emu_case "p4305 capture" -- -d $T --samples 1
    emu_options model=SPE6103 on=1
    emu_case "spe6103 group" -- -d $T -g 1 --show
    emu_case "spe6103 capture" -- -d $T --samples 1
    emu_options model=HMC8042
    emu_case "hmc8042 show" --keep-header -- -d $T --show
    emu_options model=HMC8043 on=1
    emu_case "hmc8043 group" -- -d $T -g 1 --show
    emu_case "hmc8043 capture" -- -d $T -C I2,V3 --samples 1
    emu_case "hmc8043 ocp delay" -- -d $T -g 2 --get ocp_delay
    emu_case "hmc8043 set delay" --no-output -- -d $T -g 2 --config ocp_delay=0.25 --set
    emu_options model=NGE103B on=1
    emu_case "nge103b group" -- -d $T -g 1 --show
    emu_case "nge103b fuse" -- -d $T -g 2 --get ocp_active
    emu_case "nge103b set fuse" --no-output -- -d $T -g 2 --config ocp_enabled=on --set
    emu_options model=HMP4030
    emu_case "hameg 4030 show" --keep-header -- -d $T --show
    emu_options model=HMP4040 on=1
    emu_case "hameg 4040 group" -- -d $T -g 4 --show
    emu_case "hameg 4040 regulation" -- -d $T -g 2 --get regulation
    emu_case "hameg 4040 ovp is always on" -- -d $T -g 2 --get ovp_enabled
    emu_case "hameg 4040 otp active" -- -d $T -g 2 --get otp_active
    emu_case "hameg 4040 capture" -- -d $T -C V1,V4 --samples 2
    emu_options model=HMP2020 on=1 load=1
    emu_case "hmp2020 group" -- -d $T -g 2 --show
    emu_case "hmp2020 regulation cc" -- -d $T -g 1 --get regulation
    emu_options model=HMP2030
    emu_case "hmp2030 regulation off" -- -d $T -g 3 --get regulation
    emu_options model=E36312A on=1
    emu_case "e36312a show" --keep-header -- -d $T --show
    emu_case "e36312a group" -- -d $T -g 1 --show
    emu_case "e36312a regulation" -- -d $T -g 2 --get regulation
    emu_case "e36312a otp active" -- -d $T -g 2 --get otp_active
    emu_case "e36312a ovp active" -- -d $T -g 1 --get ovp_active
    emu_case "e36312a capture" -- -d $T -C V1,I3 --samples 2
    emu_options model=E36311A
    emu_case "e36311a group" -- -d $T -g 3 --show
    emu_options model=E36312A reg=3
    emu_case "e36312a hardware failure" -- -d $T -g 1 --get regulation
    emu_options model=E36312A reg=1
    emu_case "e36312a cc" -- -d $T -g 1 --get regulation
    emu_options model=E36312A reg=0
    emu_case "e36312a unregulated" -- -d $T -g 1 --get regulation

    # Those whose outputs a probe finds. A group's show has them all, and C reads a
    # null table there.
    NO_TABLE="config_list takes ch_spec from devc->device->channels (api.c:814), which a profile with probe_channels has as NULL, so a list of a probed supply's output reads through it: sigrok-cli dies of SIGSEGV; oxrok lists what the probe found"
    emu_options model=62012P-80-60 on=1
    emu_case "62012p scan" --keep-header -- -d $T --scan
    emu_case "62012p show" --keep-header -- -d $T --show
    emu_case "62012p group" --oracle-bug "$NO_TABLE" -- -d $T -g 1 --show
    emu_case "62012p capture" -- -d $T --samples 2
    emu_case "62012p set voltage" --no-output -- -d $T -g 1 --config voltage_target=12.345 --set
    emu_options idn=CHROMA,62099P-601-1,1,2
    emu_case "62000p volts out of spec" --no-output -- -d $T --scan
    emu_options idn=CHROMA,62099P-60-121,1,2
    emu_case "62000p amps out of spec" --no-output -- -d $T --scan
    emu_options idn=CHROMA,62099P-60-120,1,2
    emu_case "62000p watts out of spec" --no-output -- -d $T --scan
    emu_options model=PS_2084-05B on=1
    emu_case "ea scan" --keep-header -- -d $T --scan
    emu_case "ea show" --keep-header -- -d $T --show
    emu_case "ea group" --oracle-bug "$NO_TABLE" -- -d $T -g 1 --show
    emu_case "ea capture" --keep-header -- -d $T --samples 2
    emu_case "ea set ovp" --no-output -- -d $T -g 1 --config ovp_threshold=50 --set
    emu_options model=PS_2084-05B on=1 event-at=2:ovp,4:ocp,6:otp
    emu_case "ea trips" --keep-header -- -d $T -C V1 --samples 7
    emu_options model=PS_2084-05B on=1 event-at=2:cc,4:cv,6:mixed
    emu_case "ea regulation" --keep-header -- -d $T -C V1 --samples 7
    emu_options model=EEZ_H24005_R3B4 on=1 channels=2
    emu_case "eez scan" --keep-header -- -d $T --scan
    emu_case "eez show" --keep-header -- -d $T --show
    emu_case "eez group" --oracle-bug "$NO_TABLE" -- -d $T -g 2 --show
    emu_case "eez capture" -- -d $T --samples 1
    emu_case "eez regulation" -- -d $T -g 1 --get regulation
    emu_options model=PSU_1.02 on=1
    emu_case "eez old scan" --keep-header -- -d $T --scan
    emu_options model=BB3_r3b3 on=1 channels=9
    emu_case "bb3 too many" --keep-header -- -d $T --show
    emu_options model=BB3_r3b3 on=1 channels=0
    emu_case "bb3 none" --keep-header -- -d $T --scan
    emu_options model=PM2813/11 on=1
    emu_case "pm2813 scan" --keep-header -- -d $T --scan
    emu_case "pm2813 show" --keep-header -- -d $T --show
    emu_case "pm2813 group" --oracle-bug "$NO_TABLE" -- -d $T -g 1 --show
    emu_case "pm2813 regulation" -- -d $T -g 2 --get regulation
    emu_case "pm2813 regulation cv" -- -d $T -g 1 --get regulation
    emu_case "pm2813 capture" -- -d $T -C V3,I3 --samples 2
    emu_options model=PM2832/01 on=1
    emu_case "pm2832 second output has no module" --keep-header -- -d $T --show
    emu_case "pm2832 capture" -- -d $T -C V2,I2 --samples 1
    emu_options idn=PHILIPS,PM2813/41,1,2
    emu_case "pm28 not in the matrix" --no-output -- -d $T --scan

    # HP 66xxB.
    emu_options model=6632B on=1
    emu_case "6632b scan" --keep-header -- -d $T --scan
    emu_case "6632b show" --keep-header -- -d $T --show
    emu_case "6632b group" -- -d $T -g 1 --show
    emu_case "6632b regulation" -- -d $T -g 1 --get regulation
    emu_case "6632b ovp active" -- -d $T -g 1 --get ovp_active
    emu_case "6632b ocp active" -- -d $T -g 1 --get ocp_active
    emu_case "6632b otp active" -- -d $T -g 1 --get otp_active
    emu_case "6632b capture" --keep-header -- -d $T --samples 2
    emu_options model=6632B on=1 event-at=2:cc,4:cv,6:ccn,8:ur
    emu_case "6632b regulation changes" --keep-header -- -d $T -C V1 --samples 9
    emu_options model=6632B on=1 event-at=2:ovp,4:clear,6:ocp,8:otp
    emu_case "6632b trips" --keep-header -- -d $T -C V1 --samples 9
    emu_options model=6632B on=1 event-at=2:idle
    emu_case "6632b idle" --keep-header -- -d $T -C V1 --samples 3
    emu_options model=6632B on=1 event-at=2:mixed
    emu_case "6632b mixed" --keep-header -- -d $T -C V1 --samples 3
    emu_options model=6632B reg=99
    emu_case "6632b regulation ur" -- -d $T -g 1 --get regulation
    emu_options model=6632B reg=1024
    emu_case "6632b regulation cc" -- -d $T -g 1 --get regulation
    emu_options model=6632B reg=2048
    emu_case "6632b regulation negative" -- -d $T -g 1 --get regulation
    emu_options model=6611C on=1
    emu_case "6611c capture" -- -d $T --samples 1
    emu_options model=66312A
    emu_case "66312a group" -- -d $T -g 1 --show

    # HP-IB: ID? and ROM?, on the same socket. COMP language on an HP 6632A.
    emu_options model=6632A on=1
    emu_case "hpib scan" --keep-header -- -d $H --scan
    emu_case "hpib show" --keep-header -- -d $H --show
    emu_case "hpib group" -- -d $H -g 1 --show
    emu_case "hpib capture" --keep-header -- -d $H --samples 2
    emu_case "hpib regulation" -- -d $H -g 1 --get regulation
    emu_case "hpib ovp active" -- -d $H -g 1 --get ovp_active
    emu_case "hpib ocp active" -- -d $H -g 1 --get ocp_active
    emu_case "hpib otp active" -- -d $H -g 1 --get otp_active
    emu_case "hpib set voltage" --no-output -- -d $H -g 1 --config voltage_target=5.5 --set
    emu_case "hpib set enabled" --no-output -- -d $H -g 1 --config enabled=on --set
    emu_case "hpib set ocp" --no-output -- -d $H -g 1 --config ocp_enabled=on --set
    emu_case "hpib set ovp threshold" --no-output -- -d $H -g 1 --config ovp_threshold=21 --set
    emu_options model=6632A on=1 event-at=2:cc,4:cv,6:ccn,8:ur
    emu_case "hpib regulation changes" --keep-header -- -d $H -C V1 --samples 9
    emu_options model=6632A on=1 event-at=2:ovp,4:ocp,6:otp
    emu_case "hpib trips" --keep-header -- -d $H -C V1 --samples 7
    emu_options model=6632A on=1 event-at=2:mixed
    emu_case "hpib mixed" --keep-header -- -d $H -C V1 --samples 3
    emu_options model=6632A event-at=2:mixed
    emu_case "hpib two states from none" --keep-header -- -d $H -C V1 --samples 3
    emu_options model=6632A reg=2
    emu_case "hpib regulation cc" -- -d $H -g 1 --get regulation
    emu_options model=6632A reg=4
    emu_case "hpib regulation ur" -- -d $H -g 1 --get regulation
    emu_options model=6632A reg=512
    emu_case "hpib regulation negative" -- -d $H -g 1 --get regulation
    emu_options model=6632A reg=8
    emu_case "hpib regulation none" -- -d $H -g 1 --get regulation
    emu_options model=6633A
    emu_case "hpib 6633a" -- -d $H -g 1 --show
    emu_options model=6634A
    emu_case "hpib 6634a" -- -d $H -g 1 --show
    emu_options model=6632A rom=junk
    emu_case "hpib no version" --keep-header -- -d $H --scan
    emu_options model=6632A hpid=HP9999
    emu_case "hpib another" --no-output -- -d $H --scan
    emu_options model=6632B hpid=HP6632B rom=A01_A02
    emu_case "hpib a 66xxb in comp language" --keep-header -- -d $H --scan
    emu_options model=6632A
    emu_case "hpib needs conn" --no-output -- -d hpib-pps --scan

    # What the instrument answers.
    emu_options idn=Acme,X,1,2
    emu_case "scan another" --no-output -- -d $T --scan
    emu_options idn=short
    emu_case "scan no idn" --no-output -- -d $T --scan
    emu_options say=RIGOL_TECHNOLOGIES,DP832,00.01
    emu_case "scan short idn" --keep-header -- -d $T --scan
    emu_options say=rigol_technologies,DP832A,1,2
    emu_case "scan by alias in lower case" --keep-header -- -d $T --scan
    emu_options say=RIGOL_TECHNOLOGIES,xDP832,1,2
    emu_case "scan a whole model" --no-output -- -d $T --scan
    emu_options mute-at=1
    emu_case "scan silent" --no-output -- -d $U --scan
    emu_options silent-at=1
    emu_case "scan silent once" --no-output -- -d $U --scan
    emu_options reg=XX
    emu_case "regulation unknown" --no-output --expect-fail -- -d $T -g 1 --get regulation

    # A setting the instrument refuses, and one that goes nowhere.
    emu_options on=1
    emu_case "a refused setting exits 0" -- -d $T -g 1 -c voltage_target=999 -C V1 --samples 1
    emu_case "a refused setting warns" --oracle-differs "$OUT_OF_RANGE" --stderr -- -d $T -g 1 -c voltage_target=999 --set
    emu_case "nan goes nowhere" --oracle-differs "$NAN_SENT" --expect-fail --stderr -- -d $T -g 1 -c voltage_target=nan --set

    # Silent, on USBTMC with its tags and on TCP with none. The scan's queries are one
    # (*IDN?), the open's one more (a beeper), so the first of the case's is the third.
    emu_options mute-at=3
    emu_case "get silent" --no-output --expect-fail -- -d $U -g 1 --get voltage_target
    emu_case "get silent tcp" --no-output --expect-fail --oracle-bug "$TCP_BLOCKS" --oracle-hangs --timeout 12 -- -d $T -g 1 --get voltage_target
    emu_case "regulation silent" --no-output --oracle-bug "$NO_VALUE" --expect-fail -- -d $U -g 1 --get regulation
    emu_case "a capture hears nothing" --stderr --oracle-bug "$NEVER_ENDS" --oracle-hangs --timeout 4 -- -d $U --time 500ms
    emu_case "a capture hears nothing tcp" --stderr --oracle-bug "$TCP_BLOCKS" --oracle-hangs --timeout 9 -- -d $T --time 500ms
    emu_options mute-at=5 on=1
    emu_case "a capture goes silent" --oracle-bug "$NEVER_ENDS" --oracle-hangs --timeout 4 -- -d $U -C V1 --time 2500ms
    emu_case "a capture goes silent tcp" --oracle-bug "$TCP_BLOCKS" --oracle-hangs --timeout 9 -- -d $T -C V1 --time 1s
    emu_options silent-at=3 on=1
    emu_case "a question lost" -- -d $U -C V1,I1 --samples 2

    # Late answers. Over TCP C's read blocks, so an answer late past 1 s is still taken,
    # and one question stays out here as well. USBTMC asks again.
    emu_options on=1 late-at=3:75
    emu_case "get late tcp" -- -d $T -g 1 --get voltage_target
    emu_case "capture late tcp" -- -d $T -C V1,I1 --samples 2
    emu_options on=1 late-at=4:75
    emu_case "capture late in a round tcp" -- -d $T -C V1,I1 --samples 2
    emu_options on=1 late-at=3:5
    emu_case "capture late" -- -d $U -C V1,I1 --samples 2
    # A time limit shorter than the instrument's gap still gets its reading.
    emu_options on=1 late-at=3:10
    emu_case "a limit shorter than a gap tcp" -- -d $T -C V1 --time 100ms

    # What is no reading.
    emu_options on=1 bad-at=2:junk
    emu_case "a reading is junk" -- -d $T -C V1,I1 --samples 2
    emu_options on=1 bad-at=2:
    emu_case "a reading is empty" --oracle-differs "$EMPTY" -- -d $T -C V1,I1 --samples 2
    emu_options on=1 bad-at=2:0x10
    emu_case "a reading is hexadecimal" --oracle-differs "$HEX" -- -d $T -C V1,I1 --samples 2
    emu_options on=1 bad-at=2:nan
    emu_case "a reading is nan" --oracle-differs "$HEX" -- -d $T -C V1,I1 --samples 2
    emu_options on=1 bad=junk
    emu_case "get junk" --no-output --expect-fail -- -d $T -g 1 --get voltage
    emu_options on=1 bad=
    emu_case "get empty" --oracle-differs "$EMPTY" --expect-fail --stderr -- -d $T -g 1 --get voltage
    emu_options model=6632B status=
    emu_case "6632b empty status" --oracle-differs "$STATUS_EMPTY" --expect-fail --stderr -- -d $T -g 1 --get regulation
    emu_options model=6632B status=junk
    emu_case "6632b junk status" --oracle-differs "$STATUS_JUNK" --expect-fail --stderr --filter "$STACK" -- -d $T -g 1 --get regulation
    emu_options model=6632B status=junk on=1
    emu_case "6632b junk in a capture" -- -d $T -C V1 --samples 1
    # What a probe finds: C has an EA's nominal values all as 0.
    emu_options model=PS_2084-05B nominal=junk
    emu_case "ea nominal junk" --keep-header -- -d $T --scan
    emu_options model=PS_2084-05B mute-at=2
    emu_case "ea voltage silent" --no-output -- -d $U --scan
    emu_options model=PS_2084-05B mute-at=3
    emu_case "ea current silent" --no-output -- -d $U --scan
    emu_options model=PS_2084-05B mute-at=4
    emu_case "ea power silent" --no-output -- -d $U --scan
    emu_options model=PSU_1.02 mute-at=2
    emu_case "eez count silent" --no-output -- -d $U --scan
    emu_options model=PSU_1.02 mute-at=3
    emu_case "eez current silent" --no-output -- -d $U --scan
    emu_options model=PSU_1.02 mute-at=4
    emu_case "eez voltage silent" --no-output -- -d $U --scan
    emu_options model=PSU_1.02 mute-at=5
    emu_case "eez power silent" --no-output -- -d $U --scan
    emu_options model=PSU_1.02 chan-info=junk
    emu_case "eez limits junk" --no-output -- -d $T --scan
    emu_options model=6632A rom=mute
    emu_case "hpib rom silent" --keep-header -- -d hpib-pps:conn=usbtmc --scan
    emu_options model=6632A mute-at=1
    emu_case "hpib silent" --no-output -- -d hpib-pps:conn=usbtmc --scan

    # A key on the list with no command in the table.
    emu_options model=PSU_1.02
    emu_case "eez ovp active has no command" --no-output --expect-fail -- -d $T -g 1 --get ovp_active

    # An interrupt, on a line kept quiet after the mark so that the count holds.
    emu_options on=1 mark-at=4 mute-at=5
    emu_case "interrupt" --oracle-bug "$SIGINT_BUG" --interrupt tcp-model --timeout 15 -- -d $T -C V1 --continuous

    # An instrument gone. On TCP it closes the connection.
    emu_options on=1
    emu_case "unplug in a capture tcp" --oracle-bug "$TCP_EOF" --timeout 4 --unplug tcp-in=4 --oxrok-fails "closed" -- -d $T -C V1 --samples 100
    emu_case "unplug in a capture" --oracle-bug "$GONE" --oracle-hangs --timeout 6 --unplug in=2:4 --oxrok-fails "unplug" -- -d $U -C V1 --samples 100
    emu_case "unplug before a set" --no-output --expect-fail --unplug in=2:2 -- -d $U -g 2 -c voltage_target=3.3 --set
}
