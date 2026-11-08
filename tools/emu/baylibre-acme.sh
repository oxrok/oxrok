# The BayLibre ACME cape's probes as the kernel's files (emu/src/models/baylibre-acme.rs),
# a tree served with --sysfs and bound over /sys/class in a user and mount namespace of
# each tool's own (tools/emu-ns.sh). Sourced by tools/diffcli.sh --emu; see docs/USB.md,
# "Scenarios". The default cape: an ina226 probe of revision A on connector 1 and one of
# revision B (an EEPROM, a 20 mΩ shunt) on 2, both with a power switch, and a tmp435 on 5.
#
# What a run holds: the files each tool opened, read and wrote (sysfs_* lines), not the
# stat(2) calls of g_file_test, which inotify doesn't see. A write the kernel would refuse
# is put back after the tool's close, so a case reads such a file in another run, not
# right after the write: the tool can't see the refusal, and a quick read can still see
# what it wrote (docs: emu/src/shells/sysfs.rs).
EMU_MODEL=baylibre-acme
EMU_SYSFS=1
EMU_WRAP="sh $TOOLS/emu-ns.sh $EMU_SYSFS_DIR/class:/sys/class --"
EMU_DRIVER=baylibre-acme
EMU_GCOV=baylibre-acme

emu_cases() {
    D=baylibre-acme
    # A capture's values in few lines: sigrok-cli prints one per channel and frame.
    VOL_ONLY="-C P1_ENRG_VOL"
    SEGV_EMPTY_HWMON="get_hwmon_index passes g_dir_read_name's NULL, for an empty hwmon directory, to sscanf (protocol.c:229), which crashes"
    TIMER_1HZ="dev_acquisition_start sets it_interval.tv_nsec to 10^9 / samplerate (api.c:280), and timerfd_settime refuses 1000000000 ns with EINVAL: no capture at 1 Hz, the bottom of its own list"
    DIV0="config_set takes a samplerate of 0 (api.c:177 checks the top only) and bl_acme_maybe_set_update_interval divides 1000 by it (protocol.c:442)"

    # Scan: which connectors have a probe, and how C numbers and names them.
    emu_case "scan" --keep-header -- -d $D --scan
    emu_case "show" --keep-header -- -d $D --show
    emu_case "show probe 1" --keep-header -- -d $D -g Probe_1 --show
    emu_case "show probe 2 revision b" --keep-header -- -d $D -g Probe_2 --show
    emu_case "show probe 5 temperature" --keep-header -- -d $D -g Probe_5 --show
    emu_options slots=ina226,ina226,ina226,ina226,ina226,ina226,ina226,ina226 revb=none pws=1,3,5,7
    emu_case "scan eight power probes" --keep-header -- -d $D --scan
    emu_case "show eight power probes 8" --keep-header -- -d $D -g Probe_8 --show
    emu_options slots=none,none,none,none,tmp435,tmp435,tmp435,tmp435 revb=none pws=none
    emu_case "scan four temperature probes" --keep-header -- -d $D --scan
    emu_options slots=empty,empty,empty,empty,empty,empty,empty,empty revb=none pws=none
    emu_case "scan nothing declared" --no-output -- -d $D --scan
    emu_options slots=none,none,none,none,none,none,none,none revb=none pws=none
    emu_case "scan chips without probes" --no-output -- -d $D --scan
    emu_options slots=other,ina226,none,none,none,none,none,none revb=none pws=none
    emu_case "scan another chip" --keep-header -- -d $D --scan
    # Revision B probes are numbered by address (revB_addr_to_num, protocol.c:307-317):
    # 0x44 on connector 3 is probe 5, 0x45 on 4 is 6, 0x42 on 5 is 3, 0x43 on 6 is 4,
    # 0x46 on 7 stays 7.
    emu_options slots=none,none,ina226,ina226,ina226,ina226,ina226,none revb=3,4,5,6,7 pws=3,6
    emu_case "scan revision b renumbered" --keep-header -- -d $D --scan
    emu_case "show revision b renumbered" --keep-header -- -d $D -g Probe_5 --show
    emu_case "get power off revision b renumbered" -- -d $D -g Probe_4 --get power_off
    # A tmp435 on connector 5 is probe 5 too: two groups of one name.
    emu_options slots=none,none,ina226,none,tmp435,none,none,none revb=3 pws=3
    emu_case "scan two probes 5" --keep-header -- -d $D --scan
    emu_options eeprom-fault=rev
    emu_case "scan eeprom of revision a" --stderr -- -d $D --scan
    emu_options eeprom-fault=type
    emu_case "get probe factor eeprom of no type" -- -d $D -g Probe_2 --get probe_factor
    emu_options eeprom-fault=short
    emu_case "get probe factor eeprom short" -- -d $D -g Probe_2 --get probe_factor
    emu_options eeprom-shunt=0
    emu_case "get probe factor eeprom shunt 0" -- -d $D -g Probe_2 --get probe_factor
    emu_options hwmon=3
    emu_case "capture hwmon numbers" -- -d $D --samples 1
    emu_options hwmon-entry=1:hwmonX
    emu_case "scan hwmon entry unnamed" --stderr -- -d $D --scan
    emu_options hwmon-entry=1:
    emu_case "scan hwmon empty" --stderr --expect-fail -- -d $D --scan
    emu_options readerr=i2c-adapter/i2c-1/1-0040/name
    emu_case "scan name unreadable" --keep-header -- -d $D --scan
    emu_options

    # The sense lines of revision A and the switches.
    emu_case "get power off" -- -d $D -g Probe_1 --get power_off
    emu_case "get power off revision b" -- -d $D -g Probe_2 --get power_off
    emu_case "set power off" -- -d $D -g Probe_1 -c power_off=on --get power_off
    emu_case "set power on" --no-output -- -d $D -g Probe_2 -c power_off=off --set
    emu_case "get power off temperature" --stderr --expect-fail -- -d $D -g Probe_5 --get power_off
    emu_case "get power off no group" --stderr --expect-fail -- -d $D --get power_off
    emu_options power=off
    emu_case "get power off when off" -- -d $D -g Probe_1 --get power_off
    emu_options pws=none
    emu_case "show probe 1 no switch" --keep-header -- -d $D -g Probe_1 --show
    emu_options pws-dir=in
    emu_case "set power off switch an input" --no-output -- -d $D -g Probe_1 -c power_off=on --set
    emu_options unexported=486
    emu_case "get power off unexported" --stderr -- -d $D -g Probe_1 --get power_off
    emu_case "set power off unexported" --stderr --expect-fail -- -d $D -g Probe_1 -c power_off=on --set
    emu_options unexported=487
    emu_case "show probe 1 sense line unexported" --stderr -- -d $D -g Probe_1 --show
    emu_options unexported=all
    emu_case "scan nothing exported" --stderr -- -d $D --scan
    emu_options value=gpio/gpio487/value:junk
    emu_case "show probe 1 sense line junk" --stderr -- -d $D -g Probe_1 --show
    emu_options absent=gpio/gpio487/value
    emu_case "show probe 1 sense line no value" --stderr -- -d $D -g Probe_1 --show
    emu_options ro=gpio/gpio486/value
    emu_case "set power off not allowed" --stderr --expect-fail -- -d $D -g Probe_1 -c power_off=on --set
    emu_options

    # The shunt resistor, mΩ to C and µΩ in the file.
    emu_case "get probe factor" -- -d $D -g Probe_1 --get probe_factor
    emu_case "get probe factor revision b" -- -d $D -g Probe_2 --get probe_factor
    emu_case "get probe factor temperature" --stderr --expect-fail -- -d $D -g Probe_5 --get probe_factor
    emu_case "get probe factor no group" --stderr --expect-fail -- -d $D --get probe_factor
    emu_case "set probe factor" -- -d $D -g Probe_1 -c probe_factor=50 --get probe_factor
    emu_case "set probe factor then capture" -- -d $D -g Probe_1 -c probe_factor=20 $VOL_ONLY,P1_ENRG_CURR,P1_ENRG_PWR --samples 1
    emu_case "set probe factor the most" --no-output -- -d $D -g Probe_1 -c probe_factor=2500 --set
    emu_case "set probe factor over" --no-output -- -d $D -g Probe_1 -c probe_factor=2501 --set
    emu_case "set probe factor 0" --no-output -- -d $D -g Probe_1 -c probe_factor=0 --set
    emu_options ro=hwmon/hwmon0/shunt_resistor
    emu_case "set probe factor not allowed" --stderr --expect-fail -- -d $D -g Probe_1 -c probe_factor=50 --set
    emu_options full=hwmon/hwmon0/shunt_resistor
    emu_case "set probe factor write fails" --no-output -- -d $D -g Probe_1 -c probe_factor=50 --set
    emu_options absent=hwmon/hwmon0/shunt_resistor
    emu_case "get probe factor no attribute" --stderr --expect-fail -- -d $D -g Probe_1 --get probe_factor
    emu_case "set probe factor no attribute" --stderr --expect-fail -- -d $D -g Probe_1 -c probe_factor=50 --set
    emu_options value=hwmon/hwmon0/shunt_resistor:abc
    emu_case "get probe factor junk" -- -d $D -g Probe_1 --get probe_factor
    emu_options readerr=hwmon/hwmon0/shunt_resistor
    emu_case "get probe factor unreadable" --stderr --expect-fail -- -d $D -g Probe_1 --get probe_factor
    emu_options value=hwmon/hwmon0/shunt_resistor:999
    emu_case "get probe factor under 1 mohm" -- -d $D -g Probe_1 --get probe_factor
    emu_options ro=hwmon/hwmon1/shunt_resistor
    emu_case "scan eeprom shunt not allowed" --stderr -- -d $D --scan
    emu_options

    # Settings of the capture.
    emu_case "get samplerate" -- -d $D --get samplerate
    emu_case "set samplerate" -- -d $D -c samplerate=50 --get samplerate
    emu_case "set samplerate the most" -- -d $D -c samplerate=500 --get samplerate
    emu_case "set samplerate over" --stderr --expect-fail -- -d $D -c samplerate=501 --get samplerate
    emu_case "set samplerate 3" --no-output -- -d $D -c samplerate=3 --set
    emu_case "get limit samples" -- -d $D --get limit_samples
    emu_case "get limit time" -- -d $D --get limit_time
    emu_options ro=hwmon/hwmon2/update_interval
    emu_case "set samplerate interval not allowed" --no-output -- -d $D -c samplerate=50 --set
    emu_options absent=hwmon/hwmon0/update_interval
    emu_case "set samplerate no interval" --no-output -- -d $D -c samplerate=50 --set
    emu_options

    # Captures. At the default 10 Hz a sample is due every 100 ms from the start.
    emu_case "capture" -- -d $D --samples 2
    emu_case "capture one channel" -- -d $D $VOL_ONLY --samples 3
    emu_case "capture temperatures" -- -d $D -C P5_TEMP_IN,P5_TEMP_OUT --samples 2
    emu_case "capture probe 2" -- -d $D -g Probe_2 --samples 1
    emu_case "capture time 250" -- -d $D $VOL_ONLY --time 250
    emu_case "capture time 50" -- -d $D $VOL_ONLY --time 50
    emu_case "capture at 50 hz" -- -d $D -c samplerate=50 $VOL_ONLY --samples 5
    emu_case "capture at 500 hz" -- -d $D -c samplerate=500 $VOL_ONLY,P1_ENRG_CURR --samples 20
    emu_case "capture at 1 hz" --stderr --expect-fail -- -d $D -c samplerate=1 $VOL_ONLY --samples 1
    emu_case "capture at 0 hz" --stderr --expect-fail -- -d $D -c samplerate=0 $VOL_ONLY --samples 1
    emu_case "capture csv" -- -d $D --samples 2 -O csv
    emu_case "capture analog" -- -d $D -C P2_ENRG_PWR --samples 2 -O analog
    # The chip's own pace: values move on every update_interval (35 ms and 125 ms at
    # power on), whatever the reads; how many reads fall in one conversion is the clock's.
    emu_options pace=time
    emu_case "capture live at 500 hz" --filter 's/: -?[0-9.]+ [mµ]?/: VALUE /' \
        --tx-filter 's/^(sysfs_read [^ ]+ -> ).*/\1VALUE/' -- -d $D -c samplerate=500 --samples 40
    emu_options

    # What a value file can hold or do.
    emu_options value=hwmon/hwmon0/in1_input:junk
    emu_case "capture junk" -- -d $D $VOL_ONLY --samples 2
    emu_options value=hwmon/hwmon0/in1_input:5000mV
    emu_case "capture number and junk" -- -d $D $VOL_ONLY --samples 1
    emu_options value=hwmon/hwmon0/in1_input:-12
    emu_case "capture negative" -- -d $D $VOL_ONLY --samples 1
    emu_options value=hwmon/hwmon0/curr1_input:-4000
    emu_case "capture negative current" -- -d $D -C P1_ENRG_CURR --samples 1
    emu_options value=hwmon/hwmon0/power1_input:409593750
    emu_case "capture largest power" -- -d $D -C P1_ENRG_PWR --samples 1
    emu_options value=hwmon/hwmon0/in1_input:
    emu_case "capture empty" -- -d $D $VOL_ONLY --samples 2
    emu_options vanish=hwmon/hwmon0/in1_input@2
    emu_case "capture value gone" -- -d $D $VOL_ONLY --samples 4
    emu_options readerr=hwmon/hwmon0/in1_input
    emu_case "capture read error" --stderr -- -d $D $VOL_ONLY,P1_ENRG_CURR --samples 2
    emu_options absent=hwmon/hwmon0/in1_input
    emu_case "capture value file absent" --stderr --expect-fail -- -d $D --samples 1
    emu_options absent=hwmon/hwmon0/in0_input
    emu_case "capture shunt voltage absent" -- -d $D $VOL_ONLY --samples 1
    emu_options
    emu_case "capture unplugged" --unplug sysfs-read=2 -- -d $D $VOL_ONLY --samples 4
}
