// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0
//
// A preload for the oracle's runs of tools/emu/mastech-ms6514.sh, which makes
// the C driver's scan able to find a device. Its call of serial_stream_detect
// passes a packet size of two frames with a buffer of two frames, and
// serial_stream_detect wants a buffer of two packets, so stock libsigrok refuses
// every scan and nothing behind it ever runs. serial_stream_detect is a hidden
// symbol, so a preload can't replace it. After sr_init this changes that one
// argument in the loaded driver, from 36 to 18, the size of a frame: the
// instruction `mov $36,%ecx` in the driver's scan, which must be there exactly
// once or the run stops. Everything else is the C driver as built, with its own
// gcov counts. Only x86-64 is served, which is what the testers are.

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

// How far into the scan function the instruction may be.
#define SCAN_BYTES 1024

static void give_up(const char *what) {
    fprintf(stderr, "mastech-ms6514-scan: %s\n", what);
    _exit(99);
}

static void change_the_size(struct driver_head *d) {
    static const unsigned char mov[] = {0xb9, 36, 0, 0, 0};
    unsigned char *code = (unsigned char *)d->scan;
    unsigned char *at = NULL;
    for (int i = 0; i < SCAN_BYTES; i++) {
        if (memcmp(code + i, mov, sizeof(mov)) == 0) {
            if (at) give_up("the instruction is there twice");
            at = code + i;
        }
    }
    if (!at) give_up("the instruction is not there");
    long page = sysconf(_SC_PAGESIZE);
    unsigned char *start = (unsigned char *)((unsigned long)at & ~(unsigned long)(page - 1));
    if (mprotect(start, 2 * (size_t)page, PROT_READ | PROT_WRITE | PROT_EXEC) != 0) give_up("mprotect");
    at[1] = 18;
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
        if (strcmp(drivers[i]->name, "mastech-ms6514") == 0) {
            change_the_size(drivers[i]);
            return ret;
        }
    }
    give_up("no such driver");
    return ret;
}
