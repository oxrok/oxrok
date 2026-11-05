// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0
//
// A preload for the oracle's run of a scenario that sets
// EMU_ORACLE_USB_FLOOR_MS (tools/diffcli.sh): a libusb bulk timeout under
// that many ms becomes that many, and nothing else changes. A C driver whose
// timeout is a few ms times out on a device that answered, whenever the host
// holds the transfer up for that long, and such a driver's scan says only
// "No devices found.". A device that does not answer still times out.
//
// libusb's header is not needed, and a tester may not have it: this is the
// start of struct libusb_transfer, which is libusb 1.0's ABI.

#define _GNU_SOURCE
#include <dlfcn.h>
#include <stdlib.h>

struct transfer_head {
    void *dev_handle;
    unsigned char flags;
    unsigned char endpoint;
    unsigned char type;
    unsigned int timeout;
};

// 0 stays 0: libusb's "no timeout".
static unsigned int with_floor(unsigned int timeout) {
    const char *value = getenv("EMU_ORACLE_USB_FLOOR_MS");
    unsigned int floor_ms = value ? (unsigned int)strtoul(value, NULL, 10) : 0;
    return timeout != 0 && timeout < floor_ms ? floor_ms : timeout;
}

int libusb_bulk_transfer(void *handle, unsigned char endpoint, unsigned char *data, int length,
                         int *transferred, unsigned int timeout) {
    static int (*real)(void *, unsigned char, unsigned char *, int, int *, unsigned int);
    if (!real) {
        *(void **)&real = dlsym(RTLD_NEXT, "libusb_bulk_transfer");
    }
    return real(handle, endpoint, data, length, transferred, with_floor(timeout));
}

int libusb_submit_transfer(struct transfer_head *transfer) {
    static int (*real)(struct transfer_head *);
    if (!real) {
        *(void **)&real = dlsym(RTLD_NEXT, "libusb_submit_transfer");
    }
    transfer->timeout = with_floor(transfer->timeout);
    return real(transfer);
}
