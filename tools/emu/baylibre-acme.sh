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
EMU_FULL=$EMU_DRIVER
EMU_GCOV=baylibre-acme

emu_cases() {
    D=baylibre-acme
    # A capture's values in few lines: sigrok-cli prints one per channel and frame.
    VOL_ONLY="-C P1_ENRG_VOL"
    # oxrok's log prefix, the driver's and its GPIO helpers' (gpio.c's own prefix), and
    # the core's refusal of a 0, which says why after C's words.
    L='s/^oxrok: (err|warn|info|dbg): oxrok::drivers::baylibre_acme(::protocol)?: /sr: baylibre-acme: /
s/^oxrok: (err|warn|info|dbg): oxrok::drivers::baylibre_acme::gpio: /sr: gpio: /
s/^oxrok: err: oxrok::core::hwdriver: /sr: hwdriver: /
s/^(Failed to set device option .samplerate.: invalid argument): it cannot be 0\.$/\1./'
    EXPORT="sr_gpio_export writes direction before export (gpio.c:67-73), and the kernel has no direction until the export, so C never exports; oxrok exports first"
    SENSE_FAILED="a sense line that can't be read is -1 to sr_gpio_getval_export, which bl_acme_register_probe takes for a power switch (protocol.c:355); oxrok says there is none"
    OFF_FAILED="read_power_state takes the -1 of a failed read for power on (protocol.c:545-546) and succeeds; oxrok exports the switch, or fails"
    STRTOL="C reads a value with strtol and no check (protocol.c:653, 494), 0 of junk; oxrok refuses what isn't the kernel's number, and a capture with no channel left to read fails"
    EMPTY_READ="read_sample takes a read of 0 bytes (protocol.c:644-653) and strtol reads its stack buffer; oxrok says so and reads that channel no more, and a capture with no channel left to read fails"
    MINUS_ONE="read_sample sends -1 for a failed read (protocol.c:645-649); oxrok sends nothing for that channel and reads it no more"
    WRITE_UNCHECKED="bl_acme_set_shunt ignores fprintf's and fclose's results (protocol.c:524-525), so a write the attribute refuses succeeds; oxrok checks them"
    STOP_UNSTARTED="a failed dev_acquisition_start leaves devc->channel NULL, and the session's stop passes it on (api.c:322-323): glib's assertions"
    SIGINT_BUG="sigrok-cli has no SIGINT handler, it dies where oxrok ends the capture"
    SEGV_EMPTY_HWMON="get_hwmon_index passes g_dir_read_name's NULL, for an empty hwmon directory, to sscanf (protocol.c:219), which crashes"
    TIMER_1HZ="dev_acquisition_start sets it_interval.tv_nsec to 10^9 / samplerate (api.c:292), and timerfd_settime refuses 1000000000 ns with EINVAL: no capture at 1 Hz, the bottom of its own list"

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
    # Revision B probes are numbered by address (revB_addr_to_num, protocol.c:310-319):
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
    emu_case "show probe 2 eeprom of revision a" --keep-header -- -d $D -g Probe_2 --show
    emu_options eeprom-fault=type
    emu_case "get probe factor eeprom of no type" -- -d $D -g Probe_2 --get probe_factor
    emu_options eeprom-fault=short
    emu_case "get probe factor eeprom short" -- -d $D -g Probe_2 --get probe_factor
    emu_options eeprom-shunt=0
    emu_case "get probe factor eeprom shunt 0" -- -d $D -g Probe_2 --get probe_factor
    emu_options hwmon=3
    emu_case "capture hwmon numbers" -- -d $D --samples 1
    emu_options hwmon-entry=1:hwmonX
    emu_case "scan hwmon entry unnamed" --stderr --filter "$L" -- -d $D --scan
    emu_options hwmon-entry=1:
    emu_case "scan hwmon empty" --stderr --filter "$L" --oracle-bug "$SEGV_EMPTY_HWMON" -- -d $D --scan
    emu_options readerr=i2c-adapter/i2c-1/1-0040/name
    emu_case "scan name unreadable" --keep-header -- -d $D --scan
    emu_options

    # The sense lines of revision A and the switches.
    emu_case "get power off" -- -d $D -g Probe_1 --get power_off
    emu_case "get power off revision b" -- -d $D -g Probe_2 --get power_off
    emu_case "set power off" -- -d $D -g Probe_1 -c power_off=on --get power_off
    emu_case "set power on" --no-output -- -d $D -g Probe_2 -c power_off=off --set
    emu_case "get power off temperature" --stderr --filter "$L" --expect-fail -- -d $D -g Probe_5 --get power_off
    emu_case "get power off no group" --stderr --filter "$L" --expect-fail -- -d $D --get power_off
    emu_options power=off
    emu_case "get power off when off" -- -d $D -g Probe_1 --get power_off
    emu_options pws=none
    emu_case "show probe 1 no switch" --keep-header -- -d $D -g Probe_1 --show
    emu_options pws-dir=in
    emu_case "set power off switch an input" --no-output -- -d $D -g Probe_1 -c power_off=on --set
    emu_options unexported=486
    emu_case "get power off unexported" --oracle-differs "$OFF_FAILED; $EXPORT" -- -d $D -g Probe_1 --get power_off
    emu_case "set power off unexported" --no-output --oracle-bug "$EXPORT" -- -d $D -g Probe_1 -c power_off=on --set
    emu_options unexported=487
    emu_case "show probe 1 sense line unexported" --keep-header --oracle-differs "$EXPORT" -- -d $D -g Probe_1 --show
    emu_options unexported=all
    emu_case "scan nothing exported" --keep-header --oracle-differs "$EXPORT" -- -d $D --scan
    emu_case "show probe 5 nothing exported" --keep-header --oracle-differs "$SENSE_FAILED; $EXPORT" -- -d $D -g Probe_5 --show
    emu_options value=gpio/gpio487/value:junk
    emu_case "show probe 1 sense line junk" --stderr --filter "$L" --oracle-differs "$SENSE_FAILED" -- -d $D -g Probe_1 --show
    emu_options absent=gpio/gpio487/value
    emu_case "show probe 1 sense line no value" --keep-header --oracle-differs "$SENSE_FAILED" -- -d $D -g Probe_1 --show
    emu_options ro=gpio/gpio486/value
    emu_case "set power off not allowed" --stderr --filter "$L" --expect-fail -- -d $D -g Probe_1 -c power_off=on --set
    emu_options

    # The shunt resistor, mΩ to C and µΩ in the file.
    emu_case "get probe factor" -- -d $D -g Probe_1 --get probe_factor
    emu_case "get probe factor revision b" -- -d $D -g Probe_2 --get probe_factor
    emu_case "get probe factor temperature" --stderr --filter "$L" --expect-fail -- -d $D -g Probe_5 --get probe_factor
    emu_case "get probe factor no group" --stderr --filter "$L" --expect-fail -- -d $D --get probe_factor
    emu_case "set probe factor" -- -d $D -g Probe_1 -c probe_factor=50 --get probe_factor
    emu_case "set probe factor then capture" -- -d $D -g Probe_1 -c probe_factor=20 $VOL_ONLY,P1_ENRG_CURR,P1_ENRG_PWR --samples 1
    emu_case "set probe factor the most" --no-output -- -d $D -g Probe_1 -c probe_factor=2500 --set
    emu_case "set probe factor over" --no-output -- -d $D -g Probe_1 -c probe_factor=2501 --set
    emu_case "set probe factor 0" --no-output -- -d $D -g Probe_1 -c probe_factor=0 --set
    emu_options ro=hwmon/hwmon0/shunt_resistor
    emu_case "set probe factor not allowed" --stderr --filter "$L" --expect-fail -- -d $D -g Probe_1 -c probe_factor=50 --set
    emu_options full=hwmon/hwmon0/shunt_resistor
    emu_case "set probe factor write fails" --stderr --filter "$L" --expect-fail --oracle-differs "$WRITE_UNCHECKED" -- -d $D -g Probe_1 -c probe_factor=50 --set
    emu_options absent=hwmon/hwmon0/shunt_resistor
    emu_case "get probe factor no attribute" --stderr --filter "$L" --expect-fail -- -d $D -g Probe_1 --get probe_factor
    emu_case "set probe factor no attribute" --stderr --filter "$L" --expect-fail -- -d $D -g Probe_1 -c probe_factor=50 --set
    emu_options value=hwmon/hwmon0/shunt_resistor:abc
    emu_case "get probe factor junk" --stderr --filter "$L" --expect-fail --oracle-differs "$STRTOL" -- -d $D -g Probe_1 --get probe_factor
    emu_options readerr=hwmon/hwmon0/shunt_resistor
    emu_case "get probe factor unreadable" --stderr --filter "$L" --expect-fail -- -d $D -g Probe_1 --get probe_factor
    emu_options value=hwmon/hwmon0/shunt_resistor:999
    emu_case "get probe factor under 1 mohm" -- -d $D -g Probe_1 --get probe_factor
    emu_options ro=hwmon/hwmon1/shunt_resistor
    emu_case "scan eeprom shunt not allowed" --stderr --filter "$L" -- -d $D --scan
    emu_options

    # Settings of the capture.
    emu_case "get samplerate" -- -d $D --get samplerate
    emu_case "set samplerate" -- -d $D -c samplerate=50 --get samplerate
    emu_case "set samplerate the most" -- -d $D -c samplerate=500 --get samplerate
    emu_case "set samplerate over" --stderr --filter "$L" --expect-fail -- -d $D -c samplerate=501 --get samplerate
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
    emu_case "capture at 1 hz" --oracle-bug "$TIMER_1HZ" -- -d $D -c samplerate=1 $VOL_ONLY --samples 1
    emu_case "capture at 0 hz" --stderr --filter "$L" --expect-fail -- -d $D -c samplerate=0 $VOL_ONLY --samples 1
    emu_case "capture csv" --filter '/^; /d' -- -d $D --samples 2 -O csv
    emu_case "capture analog" -- -d $D -C P2_ENRG_PWR --samples 2 -O analog
    # The chip's own pace: values move on every update_interval (35 ms and 125 ms at
    # power on), whatever the reads; how many reads fall in one conversion is the clock's.
    emu_options pace=time
    emu_case "capture live at 500 hz" --filter 's/: -?[0-9.]+ [mµ]?/: VALUE /' \
        --tx-filter 's/^(sysfs_read [^ ]+ -> ).*/\1VALUE/' -- -d $D -c samplerate=500 --samples 40
    emu_options

    # What a value file can hold or do.
    emu_options value=hwmon/hwmon0/in1_input:junk
    NOREAD="No channel can be read any more"
    emu_case "capture junk" --stderr --filter "$L" --oracle-differs "$STRTOL" --oxrok-fails "$NOREAD" -- -d $D $VOL_ONLY --samples 2
    emu_options value=hwmon/hwmon0/in1_input:5000mV
    emu_case "capture number and junk" --stderr --filter "$L" --oracle-differs "$STRTOL" --oxrok-fails "$NOREAD" -- -d $D $VOL_ONLY --samples 1
    emu_options value=hwmon/hwmon0/in1_input:-12
    emu_case "capture negative" -- -d $D $VOL_ONLY --samples 1
    emu_options value=hwmon/hwmon0/curr1_input:-4000
    emu_case "capture negative current" -- -d $D -C P1_ENRG_CURR --samples 1
    emu_options value=hwmon/hwmon0/power1_input:409593750
    emu_case "capture largest power" -- -d $D -C P1_ENRG_PWR --samples 1
    emu_options value=hwmon/hwmon0/in1_input:
    emu_case "capture empty" --stderr --filter "$L" --oracle-differs "$EMPTY_READ" --oxrok-fails "$NOREAD" -- -d $D $VOL_ONLY --samples 2
    emu_options vanish=hwmon/hwmon0/in1_input@2
    emu_case "capture value gone" --oracle-differs "$EMPTY_READ" --oxrok-fails "$NOREAD" -- -d $D $VOL_ONLY --samples 4
    emu_options readerr=hwmon/hwmon0/in1_input
    emu_case "capture read error" --oracle-differs "$MINUS_ONE" -- -d $D $VOL_ONLY,P1_ENRG_CURR --samples 2
    emu_case "capture read error says so" --stderr --filter "$L" -- -d $D $VOL_ONLY,P1_ENRG_CURR --samples 2
    emu_options absent=hwmon/hwmon0/in1_input
    emu_case "capture value file absent" --stderr --filter "$L" --expect-fail --oracle-bug "$STOP_UNSTARTED" -- -d $D --samples 1
    emu_options absent=hwmon/hwmon0/in0_input
    emu_case "capture shunt voltage absent" -- -d $D $VOL_ONLY --samples 1
    emu_options
    emu_case "capture unplugged" --unplug sysfs-read=2 --oracle-differs "$EMPTY_READ" --oxrok-fails "$NOREAD" -- -d $D $VOL_ONLY --samples 4
    # At 2 Hz: the session looks at its stop every 100 ms, and the harness at the
    # event every 20 ms, well inside a period.
    emu_case "capture interrupted" --interrupt sysfs-read=3 --oracle-bug "$SIGINT_BUG" -- -d $D -c samplerate=2 $VOL_ONLY --continuous
}
