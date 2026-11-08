// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0
//
// A preload for the oracle's run of a scenario whose C driver sends no
// SR_DF_HEADER: sigrok-cli's datafeed_in drops every packet that comes before
// one (session.c:285-287), so such a capture prints nothing and exits 0. This
// hands the datafeed callback a header just ahead of the first packet the
// driver sends, unless that packet is already one, and changes nothing else:
// the driver and the session run as built. tools/diffcli.sh builds it once per
// run, and a scenario takes it through emu_header_oracle.
//
// Limits: it keeps the last callback added, as sigrok-cli and libsigrok-run add
// one; and it sends one header per process, as each runs one capture.
//
// libsigrok's header is not needed: these are libsigrok 0.5's datafeed ABI.

#define _GNU_SOURCE
#include <dlfcn.h>
#include <stdint.h>
#include <sys/time.h>

#define SR_DF_HEADER 10000

struct packet {
    uint16_t type;
    const void *payload;
};

struct header {
    int feed_version;
    struct timeval starttime;
};

typedef void (*feed)(const void *sdi, const struct packet *p, void *data);

static feed real_cb;
static int headed;

static void wrapped(const void *sdi, const struct packet *p, void *data) {
    if (!headed) {
        struct header h = {1, {0, 0}};
        struct packet first = {SR_DF_HEADER, &h};
        headed = 1;
        if (p->type != SR_DF_HEADER) {
            gettimeofday(&h.starttime, 0);
            real_cb(sdi, &first, data);
        }
    }
    real_cb(sdi, p, data);
}

int sr_session_datafeed_callback_add(void *session, feed cb, void *data) {
    static int (*real)(void *, feed, void *);
    if (!real)
        *(void **)&real = dlsym(RTLD_NEXT, "sr_session_datafeed_callback_add");
    real_cb = cb;
    return real(session, wrapped, data);
}
