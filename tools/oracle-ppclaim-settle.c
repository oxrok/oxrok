// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0
//
// A preload for the oracle's run of a scenario that sets
// EMU_ORACLE_PPCLAIM_SETTLE_MS (tools/diffcli.sh): after each ppdev
// PPCLAIM that succeeds, the caller sleeps that many ms, and nothing else
// changes. The kernel's uss720 ends a claim with a register read it does
// not wait for, whose late answer overwrites its soft copy of the control
// register: a PPDATADIR right after the claim is then lost from the next
// PPWCONTROL. libieee1284 issues one at once, so on a USB/IP cable the
// oracle's transactions vary from run to run; oxrok waits as long after
// its own claims (src/parport/mod.rs, CLAIM_SETTLE).

#define _GNU_SOURCE
#include <dlfcn.h>
#include <stdarg.h>
#include <stdlib.h>
#include <time.h>

// _IO('p', 0x8b), include/uapi/linux/ppdev.h.
#define PPCLAIM 0x708bUL

int ioctl(int fd, unsigned long request, ...) {
    static int (*real)(int, unsigned long, ...);
    if (!real) {
        *(void **)&real = dlsym(RTLD_NEXT, "ioctl");
    }
    va_list ap;
    va_start(ap, request);
    void *arg = va_arg(ap, void *);
    va_end(ap);
    int ret = real(fd, request, arg);
    if (ret == 0 && request == PPCLAIM) {
        const char *value = getenv("EMU_ORACLE_PPCLAIM_SETTLE_MS");
        long ms = value ? strtol(value, NULL, 10) : 0;
        if (ms > 0) {
            struct timespec t = { ms / 1000, (ms % 1000) * 1000000L };
            nanosleep(&t, NULL);
        }
    }
    return ret;
}
