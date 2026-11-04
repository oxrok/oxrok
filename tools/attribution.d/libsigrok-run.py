# The libsigrok runner of tools/diffcli.sh (tools/libsigrok-run.c); see tools/attribution.py.
MAP["tools/libsigrok-run.c"] = (
    ["sigrok-cli/main.c (the logger, select_channels, the maybe_config helpers)",
     "sigrok-cli/device.c (device_scan)",
     "sigrok-cli/parsers.c (the option, channel and trigger parsers)",
     "sigrok-cli/session.c (run_session, datafeed_in, opt_to_gvar)",
     "sigrok-cli/show.c (show_dev_list)"], INDEPENDENT)
