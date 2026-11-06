# Bench supplies (emu/src/models/scpi-pps.rs), scpi-pps and hpib-pps, on USBTMC and TCP.
# Sourced by tools/diffcli.sh --emu; see docs/USB.md, "Writing a SCPI model".
EMU_MODEL=scpi-pps
EMU_TMC=1
EMU_TCP=1
EMU_DRIVER="scpi-pps hpib-pps"
EMU_GCOV=scpi-pps
EMU_GCOV_SHARED="src/scpi/scpi.c src/scpi/scpi_usbtmc_libusb.c src/scpi/scpi_tcp.c src/tcp.c"

emu_cases() {
    U=scpi-pps
    T=scpi-pps:conn=tcp-raw/127.0.0.1/$EMU_TCP_PORT
    H=hpib-pps:conn=tcp-raw/127.0.0.1/$EMU_TCP_PORT

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
    emu_case "set points then on" --no-output -- -d $T -g 2 --config enabled=on:voltage_target=3.3:current_limit=0.5 --set
    # The output comes on after its set points, and a capture reads what they did.
    emu_case "capture" -- -d $U --samples 2
    emu_case "capture tcp" -- -d $T --samples 2
    emu_case "capture analog" -- -d $T --samples 1 -O analog
    emu_case "capture one channel" -- -d $T -C I2 --samples 3
    emu_case "capture two outputs" -- -d $T -C V1,V3 --samples 2
    emu_case "capture over a set" -- -d $T -g 1 -c voltage_target=5:current_limit=0.5:enabled=on --samples 2
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
    emu_options model=E36312A reg=0
    emu_case "e36312a unregulated" -- -d $T -g 1 --get regulation

    # Those whose outputs a probe finds. A group's show has them all, and C reads a
    # null table there.
    NO_TABLE="config_list takes ch_spec from devc->device->channels (api.c:814), which a profile with probe_channels has as NULL, so a list of a probed supply's output reads through it: sigrok-cli dies of SIGSEGV; oxrok lists what the probe found"
    emu_options model=62012P-80-60 on=1
    emu_case "62012p scan" --keep-header -- -d $T --scan
    emu_case "62012p show" --keep-header -- -d $T --show
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
    emu_case "ea capture" --keep-header -- -d $T --samples 2
    emu_case "ea set ovp" --no-output -- -d $T -g 1 --config ovp_threshold=50 --set
    emu_options model=PS_2084-05B on=1 event-at=2:ovp,4:ocp,6:otp
    emu_case "ea trips" --keep-header -- -d $T -C V1 --samples 7
    emu_options model=PS_2084-05B on=1 event-at=2:cc,4:cv,6:idle
    emu_case "ea regulation" --keep-header -- -d $T -C V1 --samples 7
    emu_options model=EEZ_H24005_R3B4 on=1 channels=2
    emu_case "eez scan" --keep-header -- -d $T --scan
    emu_case "eez show" --keep-header -- -d $T --show
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
    emu_case "pm2813 regulation" -- -d $T -g 2 --get regulation
    emu_case "pm2813 capture" -- -d $T -C V3,I3 --samples 2
    emu_options model=PM2832/01 on=1
    emu_case "pm2832 second output has no module" --keep-header -- -d $T --show
    emu_case "pm2832 capture" -- -d $T -C V2,I2 --samples 1
    emu_options idn=PHILIPS,PM2834/01,1,2
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
}
