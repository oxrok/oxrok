// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0
//
// A preload for the oracle's runs of tools/emu/teleinfo.sh, which makes the C
// driver's scan able to find a device. Its call of serial_stream_detect passes a
// packet size of 292 with a buffer of 292, and serial_stream_detect has wanted a
// buffer of two packets since 2019 (d03815a0, which turned `maxlen < packet_size
// / 2` into `* 2`), so stock libsigrok refuses every scan and nothing behind it
// runs. That function is a hidden symbol, so a preload can't replace it. After
// sr_init this takes that one check out in the loaded library: the driver's scan
// names the function in a call right after it pushes its timeout of 3000, and in
// the function the check's `jae` becomes a `jmp`. Each pattern must be there
// exactly once, or the run stops. So the scan works as it did up to 0.5.1: it
// reads 292 bytes, then looks at them once. Everything else is the C as built,
// with its own gcov counts. Only x86-64 and -O0's code are served, which is what
// the testers' oracle is.

#define _GNU_SOURCE
#include <dlfcn.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>

// The start of struct sr_dev_driver (libsigrok.h), whose header needs glib's.
struct driver_head {
    const char *name;
    const char *longname;
    int api_version;
    int (*init)(void);
    int (*cleanup)(void);
    void *(*scan)(void);
};

// How far into the scan function the call may be.
#define SCAN_BYTES 1024
// How far after the pushes the call may be.
#define CALL_BYTES 32
// How far into serial_stream_detect the check may be.
#define CHECK_BYTES 256

static void give_up(const char *what) {
    fprintf(stderr, "teleinfo-scan: %s\n", what);
    _exit(99);
}

// The only place of `pattern` in `size` bytes from `code`.
static unsigned char *only(unsigned char *code, size_t size, const unsigned char *pattern, size_t len, const char *what) {
    unsigned char *at = NULL;
    for (size_t i = 0; i + len <= size; i++) {
        if (memcmp(code + i, pattern, len) == 0) {
            if (at) give_up(what);
            at = code + i;
        }
    }
    if (!at) give_up(what);
    return at;
}

static void take_the_check_out(struct driver_head *d) {
    // push $3000; push $0, the last two arguments of the call.
    static const unsigned char pushes[] = {0x68, 0xb8, 0x0b, 0, 0, 0x6a, 0};
    // add %rax,%rax; cmp %rax,-0x18(%rbp); jae: `max_fill_idx < 2 * packet_size`.
    static const unsigned char check[] = {0x48, 0x01, 0xc0, 0x48, 0x39, 0x45, 0xe8, 0x73};
    unsigned char *scan = (unsigned char *)d->scan;
    unsigned char *at = only(scan, SCAN_BYTES, pushes, sizeof(pushes), "the scan's pushes are not there once");
    unsigned char *call = memchr(at + sizeof(pushes), 0xe8, CALL_BYTES);
    if (!call) give_up("no call after the scan's pushes");
    int rel;
    memcpy(&rel, call + 1, sizeof(rel));
    unsigned char *detect = call + 5 + rel;
    unsigned char *jump = only(detect, CHECK_BYTES, check, sizeof(check), "the check is not there once") + sizeof(check) - 1;
    long page = sysconf(_SC_PAGESIZE);
    unsigned char *start = (unsigned char *)((unsigned long)jump & ~(unsigned long)(page - 1));
    if (mprotect(start, 2 * (size_t)page, PROT_READ | PROT_WRITE | PROT_EXEC) != 0) give_up("mprotect");
    *jump = 0xeb;
    if (mprotect(start, 2 * (size_t)page, PROT_READ | PROT_EXEC) != 0) give_up("mprotect");
}

int sr_init(void **ctx) {
    int (*real)(void **) = (int (*)(void **))dlsym(RTLD_NEXT, "sr_init");
    struct driver_head **(*list)(void *) = (struct driver_head **(*)(void *))dlsym(RTLD_NEXT, "sr_driver_list");
    if (!real || !list) give_up("no sr_init or sr_driver_list");
    int ret = real(ctx);
    if (ret != 0) return ret;
    struct driver_head **drivers = list(*ctx);
    for (int i = 0; drivers && drivers[i]; i++) {
        if (strcmp(drivers[i]->name, "teleinfo") == 0) {
            take_the_check_out(drivers[i]);
            return ret;
        }
    }
    give_up("no such driver");
    return ret;
}
