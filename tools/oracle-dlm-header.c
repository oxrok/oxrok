// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

/*
 * A preload for yokogawa-dlm's C driver under sigrok-cli (tools/emu/yokogawa-dlm.sh).
 *
 * dev_acquisition_start (yokogawa-dlm/api.c) never calls std_session_send_df_header,
 * and sigrok-cli's datafeed_in skips every packet before a header (session.c:286), so
 * a stock capture prints nothing and exits 0. This gives sigrok-cli's callback the one
 * header the driver doesn't send, right before the first packet the driver does send.
 * The driver, the session and the output modules run unchanged, on what they are sent;
 * nothing else is touched.
 */
#define _GNU_SOURCE
#include <dlfcn.h>
#include <stdint.h>
#include <stdlib.h>
#include <sys/time.h>

#define SR_DF_HEADER 10000

struct sr_datafeed_packet {
	uint16_t type;
	const void *payload;
};

struct sr_datafeed_header {
	int feed_version;
	struct timeval starttime;
};

typedef void (*datafeed_cb)(const void *sdi, const struct sr_datafeed_packet *packet,
		void *cb_data);

static datafeed_cb client;
static int said;

static void with_header(const void *sdi, const struct sr_datafeed_packet *packet,
		void *cb_data)
{
	if (!said) {
		struct sr_datafeed_header header = { .feed_version = 1 };
		struct sr_datafeed_packet first = { .type = SR_DF_HEADER, .payload = &header };

		said = 1;
		gettimeofday(&header.starttime, NULL);
		if (packet->type != SR_DF_HEADER)
			client(sdi, &first, cb_data);
	}
	client(sdi, packet, cb_data);
}

int sr_session_datafeed_callback_add(void *session, datafeed_cb cb, void *cb_data)
{
	static int (*real)(void *, datafeed_cb, void *);

	if (!real)
		real = (int (*)(void *, datafeed_cb, void *))dlsym(RTLD_NEXT,
				"sr_session_datafeed_callback_add");
	client = cb;
	return real(session, with_header, cb_data);
}
