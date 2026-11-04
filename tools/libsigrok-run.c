// Counterpart in sigrok:
//   sigrok-cli/main.c (the logger, select_channels, the maybe_config helpers),
//   sigrok-cli/device.c (device_scan),
//   sigrok-cli/parsers.c (the option, channel and trigger parsers),
//   sigrok-cli/session.c (run_session, datafeed_in, opt_to_gvar),
//   sigrok-cli/show.c (show_dev_list),
//   Copyright (C) 2013 Bert Vermeulen <bert@biot.com>,
//   Copyright (C) 2011 Bert Vermeulen <bert@biot.com>,
//   with commits by Uwe Hermann, Gerhard Sittig, Martin Ling, Daniel Elstner,
//   Karl Palsson, Dan Horák, Devan Lai, Jens Steinhauser, Alexandru Gagniuc,
//   Anatoly Sokolov, poljar (Damir Jelić), Bartosz Golaszewski, Soeren Apel,
//   Aleksander Alekseev, Aurelien Jacobs, Guido Trentalancia, Peter Zotov,
//   Vincent Palatin, Andreas Sandberg, Frank Stettner
//   (sigrok-cli sigrok-cli-unreleased-103-gf44dd91).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

/*
 * libsigrok-run: drives libsigrok as sigrok-cli does, for drivers that
 * sigrok-cli cannot drive. sigrok-cli runs a session only with
 * --samples, --time, --frames or --continuous, and refuses the session
 * when the driver offers none of them; some drivers (lecroy-logicstudio)
 * end each capture on their own and offer no limit at all. This runner
 * takes sigrok-cli's syntax and output, and calls sr_session_start()
 * itself, with or without a limit:
 *
 *   libsigrok-run -d DRIVER[:KEY=VALUE...] [-l LEVEL] [-c KEY=VALUE[:...]]...
 *                 [-C CHANNELS] [-t TRIGGER] [-O FORMAT[:KEY=VALUE...]]
 *                 [--samples N] [--time T] [--frames N] [--stop-after N]
 *   libsigrok-run -d DRIVER[:KEY=VALUE...] [-l LEVEL] --scan
 *
 * The options mean what they mean to sigrok-cli, and the common failures
 * read the same (a --samples the driver cannot take is "Failed to
 * configure sample limit."). It is a subset, not a copy: no -g, -o,
 * --continuous, --get, --set, --show, decoders or transforms, no
 * channel_group= in -c, and a scan that finds several devices is
 * refused rather than narrowed to the one that is not a demo device.
 * With none of --samples, --time and --frames the session runs until the
 * driver ends it.
 *
 * --stop-after N is a sample limit for a driver that has none. The runner
 * counts the samples itself (logic ones, else those of the first analog
 * channel), cuts the packet that reaches N and drops what comes after,
 * and asks for the stop from an idle callback of the main loop, as a
 * client's key press or button would: sr_session_stop() from the
 * datafeed callback would run the driver's stop inside its transfer
 * callback, inside libusb's event handling. So a driver that ends the
 * capture in the same callback as its samples (lecroy-logicstudio) never
 * sees that stop, and the option only cuts its output. --stop-after 0
 * asks for the stop as soon as the session has started, at the header
 * packet, before any sample.
 *
 * The output goes through libsigrok's output modules (default bits, as
 * sigrok-cli without -o), so it reads as sigrok-cli's would.
 *
 * tools/diffcli.sh builds it against the oracle's libsigrok (emu_case
 * --libsigrok, docs/USB.md): it links the same libsigrok.so as
 * sigrok-cli, so a --coverage build counts its runs in the same .gcda
 * files.
 */

#include <glib.h>
#include <inttypes.h>
#include <libsigrok/libsigrok.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int opt_loglevel = SR_LOG_WARN;
static char *opt_drv;
static GPtrArray *opt_configs;
static char *opt_channels;
static char *opt_triggers;
static char *opt_output_format;
static char *opt_samples;
static char *opt_time;
static char *opt_frames;
static char *opt_stop_after;
static gboolean opt_scan;

static struct sr_context *sr_ctx;
static struct sr_session *session;
static uint64_t limit_samples;
static uint64_t stop_after;
static gboolean have_stop;

/* sigrok-cli's logger: everything to stderr, a critical ends the run. */
static void logger(const gchar *log_domain, GLogLevelFlags log_level,
		   const gchar *message, gpointer cb_data)
{
	(void)log_domain;
	(void)cb_data;

	if (log_level & (G_LOG_LEVEL_ERROR | G_LOG_LEVEL_CRITICAL | G_LOG_LEVEL_WARNING)
			|| opt_loglevel > SR_LOG_WARN) {
		fprintf(stderr, "%s\n", message);
		fflush(stderr);
	}
	if (log_level & (G_LOG_LEVEL_ERROR | G_LOG_LEVEL_CRITICAL))
		exit(1);
}

static int maybe_config_get(const struct sr_dev_inst *sdi, uint32_t key, GVariant **gvar)
{
	if (sr_dev_config_capabilities_list(sdi, NULL, key) & SR_CONF_GET)
		return sr_config_get(sr_dev_inst_driver_get(sdi), sdi, NULL, key, gvar);
	return SR_ERR_NA;
}

static int maybe_config_set(const struct sr_dev_inst *sdi, uint32_t key, GVariant *gvar)
{
	if (sr_dev_config_capabilities_list(sdi, NULL, key) & SR_CONF_SET)
		return sr_config_set(sdi, NULL, key, gvar);
	return SR_ERR_NA;
}

static int maybe_config_list(const struct sr_dev_inst *sdi, uint32_t key, GVariant **gvar)
{
	if (sr_dev_config_capabilities_list(sdi, NULL, key) & SR_CONF_LIST)
		return sr_config_list(sr_dev_inst_driver_get(sdi), sdi, NULL, key, gvar);
	return SR_ERR_NA;
}

/* parsers.c's parse_generic_arg(), the ID form or the plain one. */
static GHashTable *parse_generic_arg(const char *arg, gboolean sep_first)
{
	GHashTable *hash;
	char **elements, *eq;
	int i;

	if (!arg || !arg[0])
		return NULL;
	hash = g_hash_table_new_full(g_str_hash, g_str_equal, g_free, g_free);
	elements = g_strsplit(arg, ":", 0);
	i = 0;
	if (sep_first)
		g_hash_table_insert(hash, g_strdup("sigrok_key"), g_strdup(elements[i++]));
	for (; elements[i]; i++) {
		if (!elements[i][0])
			continue;
		eq = strchr(elements[i], '=');
		if (eq)
			*eq++ = '\0';
		g_hash_table_insert(hash, g_strdup(elements[i]), eq ? g_strdup(eq) : NULL);
	}
	g_strfreev(elements);
	return hash;
}

/* session.c's opt_to_gvar(), for the types drivers take. */
static int opt_to_gvar(const char *key, const char *value, struct sr_config *src)
{
	const struct sr_key_info *srci, *srmqi;
	double dlow, dhigh;
	uint64_t tmp_u64, p, q, low, high, mqflags;
	GVariant *pair[2];
	GVariantBuilder *vbl;
	gchar **parts;
	int ret;

	if (!(srci = sr_key_info_name_get(SR_KEY_CONFIG, key))) {
		g_critical("Unknown device option '%s'.", key);
		return -1;
	}
	src->key = srci->key;
	if ((!value || strlen(value) == 0) && srci->datatype != SR_T_BOOL) {
		g_critical("Option '%s' needs a value.", key);
		return -1;
	}
	ret = 0;
	switch (srci->datatype) {
	case SR_T_UINT64:
		if ((ret = sr_parse_sizestring(value, &tmp_u64)) == 0)
			src->data = g_variant_new_uint64(tmp_u64);
		break;
	case SR_T_INT32:
		if ((ret = sr_parse_sizestring(value, &tmp_u64)) == 0)
			src->data = g_variant_new_int32(tmp_u64);
		break;
	case SR_T_STRING:
		src->data = g_variant_new_string(value);
		break;
	case SR_T_BOOL:
		src->data = g_variant_new_boolean(value ? sr_parse_boolstring(value) : TRUE);
		break;
	case SR_T_FLOAT:
		src->data = g_variant_new_double(strtof(value, NULL));
		break;
	case SR_T_RATIONAL_PERIOD:
	case SR_T_RATIONAL_VOLT:
		if (srci->datatype == SR_T_RATIONAL_PERIOD)
			ret = sr_parse_period(value, &p, &q);
		else
			ret = sr_parse_voltage(value, &p, &q);
		if (ret != SR_OK)
			break;
		pair[0] = g_variant_new_uint64(p);
		pair[1] = g_variant_new_uint64(q);
		src->data = g_variant_new_tuple(pair, 2);
		break;
	case SR_T_UINT64_RANGE:
		if (sscanf(value, "%" SCNu64 "-%" SCNu64, &low, &high) != 2) {
			ret = -1;
			break;
		}
		pair[0] = g_variant_new_uint64(low);
		pair[1] = g_variant_new_uint64(high);
		src->data = g_variant_new_tuple(pair, 2);
		break;
	case SR_T_DOUBLE_RANGE:
		if (sscanf(value, "%lf-%lf", &dlow, &dhigh) != 2) {
			ret = -1;
			break;
		}
		pair[0] = g_variant_new_double(dlow);
		pair[1] = g_variant_new_double(dhigh);
		src->data = g_variant_new_tuple(pair, 2);
		break;
	case SR_T_KEYVALUE:
		parts = g_strsplit(value, "=", 2);
		if (!parts[0] || !parts[1]) {
			ret = -1;
		} else {
			vbl = g_variant_builder_new(G_VARIANT_TYPE_DICTIONARY);
			g_variant_builder_add(vbl, "{ss}", parts[0], parts[1]);
			src->data = g_variant_builder_end(vbl);
		}
		g_strfreev(parts);
		break;
	case SR_T_MQ:
		parts = g_strsplit(value, "/", 0);
		if (!parts[0] || !(srmqi = sr_key_info_name_get(SR_KEY_MQ, parts[0]))) {
			g_strfreev(parts);
			ret = -1;
			break;
		}
		pair[0] = g_variant_new_uint32(srmqi->key);
		mqflags = 0;
		for (int i = 1; parts[i]; i++) {
			if (!(srmqi = sr_key_info_name_get(SR_KEY_MQFLAGS, parts[i]))) {
				ret = -1;
				break;
			}
			mqflags |= srmqi->key;
		}
		g_strfreev(parts);
		if (ret != -1) {
			pair[1] = g_variant_new_uint64(mqflags);
			src->data = g_variant_new_tuple(pair, 2);
		}
		break;
	default:
		g_critical("Unknown data type specified for option '%s' "
			"(driver implementation bug?).", key);
		ret = -1;
	}
	if (ret < 0)
		g_critical("Invalid value: '%s' for option '%s'", value, key);
	return ret;
}

/* parsers.c's parse_driver() and device.c's device_scan(). */
static GSList *device_scan(void)
{
	struct sr_dev_driver **drivers, *driver = NULL;
	GHashTable *drvargs;
	GHashTableIter iter;
	gpointer k, v;
	GSList *drvopts = NULL, *devices;
	struct sr_config *src;
	const char *name;
	int i;

	drvargs = parse_generic_arg(opt_drv, TRUE);
	name = g_hash_table_lookup(drvargs, "sigrok_key");
	drivers = sr_driver_list(sr_ctx);
	for (i = 0; drivers[i]; i++)
		if (!strcmp(drivers[i]->name, name))
			driver = drivers[i];
	if (!driver)
		g_critical("Driver %s not found.", name);
	if (sr_driver_init(sr_ctx, driver) != SR_OK)
		g_critical("Failed to initialize driver.");
	g_hash_table_iter_init(&iter, drvargs);
	while (g_hash_table_iter_next(&iter, &k, &v)) {
		if (!strcmp(k, "sigrok_key"))
			continue;
		src = g_malloc0(sizeof(*src));
		if (opt_to_gvar(k, v, src) != 0)
			return NULL;
		drvopts = g_slist_append(drvopts, src);
	}
	g_hash_table_destroy(drvargs);
	devices = sr_driver_scan(driver, drvopts);
	for (GSList *l = drvopts; l; l = l->next) {
		src = l->data;
		g_variant_unref(src->data);
		g_free(src);
	}
	g_slist_free(drvopts);
	return devices;
}

static gint sort_channels(gconstpointer a, gconstpointer b)
{
	const struct sr_channel *pa = a, *pb = b;

	return pa->index - pb->index;
}

/* show.c's show_dev_list(). */
static void show_dev_list(void)
{
	GSList *devices, *l, *sl, *c;
	GVariant *gvar;
	GString *s;
	const char *fields[3];
	const char *sernum;

	if (!(devices = device_scan()))
		return;
	printf("The following devices were found:\n");
	for (l = devices; l; l = l->next) {
		struct sr_dev_inst *sdi = l->data;
		GSList *channels = sr_dev_inst_channels_get(sdi);

		s = g_string_new(sr_dev_inst_driver_get(sdi)->name);
		if (maybe_config_get(sdi, SR_CONF_CONN, &gvar) == SR_OK) {
			g_string_append_printf(s, ":conn=%s", g_variant_get_string(gvar, NULL));
			g_variant_unref(gvar);
		}
		g_string_append(s, " - ");
		fields[0] = sr_dev_inst_vendor_get(sdi);
		fields[1] = sr_dev_inst_model_get(sdi);
		fields[2] = sr_dev_inst_version_get(sdi);
		for (int i = 0; i < 3; i++)
			if (fields[i] && fields[i][0])
				g_string_append_printf(s, "%s ", fields[i]);
		sernum = sr_dev_inst_sernum_get(sdi);
		if (sernum && sernum[0])
			g_string_append_printf(s, "[S/N: %s] ", sernum);
		if (channels && !channels->next) {
			g_string_append_printf(s, "with 1 channel: %s",
				((struct sr_channel *)channels->data)->name);
		} else if (channels) {
			sl = g_slist_sort(g_slist_copy(channels), sort_channels);
			g_string_append_printf(s, "with %d channels:", g_slist_length(sl));
			for (c = sl; c; c = c->next)
				g_string_append_printf(s, " %s", ((struct sr_channel *)c->data)->name);
			g_slist_free(sl);
		}
		printf("%s\n", s->str);
		g_string_free(s, TRUE);
	}
	g_slist_free(devices);
}

static struct sr_channel *find_channel(GSList *channels, const char *name)
{
	for (GSList *l = channels; l; l = l->next)
		if (!strcmp(((struct sr_channel *)l->data)->name, name))
			return l->data;
	return NULL;
}

/* main.c's select_channels() with parsers.c's parse_channelstring():
 * names, renames, and A-B ranges of channels named as numbers. */
static void select_channels(struct sr_dev_inst *sdi)
{
	GSList *channels = sr_dev_inst_channels_get(sdi), *selected = NULL, *l;
	char **tokens, **names, **range, *end, str[12];
	struct sr_channel *ch;
	long b, e;

	if (!opt_channels)
		return;
	/* An empty list is all channels, enabled. */
	if (!opt_channels[0])
		selected = g_slist_copy(channels);
	tokens = g_strsplit(opt_channels, ",", 0);
	for (int i = 0; tokens[i]; i++) {
		if (!tokens[i][0])
			g_critical("Invalid empty channel.");
		if (strchr(tokens[i], '-')) {
			/* As parse_channelstring(): the end of the range is
			 * read up to what is not a digit, so 1-2-3 is 1-2. */
			range = g_strsplit(tokens[i], "-", 2);
			b = strtol(range[0], &end, 10);
			if (end == range[0] || *end)
				g_critical("Invalid channel '%s'.", range[0]);
			e = strtol(range[1], NULL, 10);
			if (b < 0 || b >= e)
				g_critical("Invalid channel range '%s'.", tokens[i]);
			g_strfreev(range);
			for (; b <= e; b++) {
				snprintf(str, sizeof(str), "%ld", b);
				if (!(ch = find_channel(channels, str)))
					g_critical("unknown channel '%ld'.", b);
				selected = g_slist_append(selected, ch);
			}
			continue;
		}
		names = g_strsplit(tokens[i], "=", 2);
		ch = find_channel(channels, names[0]);
		if (!ch)
			g_critical("unknown channel '%s'.", names[0]);
		if (names[1])
			sr_dev_channel_name_set(ch, names[1]);
		selected = g_slist_append(selected, ch);
		g_strfreev(names);
	}
	g_strfreev(tokens);
	for (l = channels; l; l = l->next)
		if (sr_dev_channel_enable(l->data, g_slist_find(selected, l->data) != NULL) != SR_OK)
			g_critical("Failed to set channels.");
	g_slist_free(selected);
}

/* parsers.c's parse_triggerstring(). */
static struct sr_trigger *parse_triggerstring(const struct sr_dev_inst *sdi, const char *s)
{
	struct sr_trigger *trigger;
	struct sr_trigger_stage *stage;
	struct sr_channel *ch;
	GVariant *gvar;
	gsize num_matches;
	const int32_t *matches;
	char **tokens, *sep;
	GSList *l;
	int32_t match;
	gboolean found;

	if (maybe_config_list(sdi, SR_CONF_TRIGGER_MATCH, &gvar) != SR_OK)
		g_critical("Device doesn't support any triggers.");
	matches = g_variant_get_fixed_array(gvar, &num_matches, sizeof(int32_t));
	trigger = sr_trigger_new(NULL);
	tokens = g_strsplit(s, ",", -1);
	for (int i = 0; tokens[i]; i++) {
		if (!(sep = strchr(tokens[i], '=')))
			g_critical("Invalid trigger '%s'.", tokens[i]);
		*sep++ = 0;
		ch = NULL;
		for (l = sr_dev_inst_channels_get(sdi); l; l = l->next)
			if (((struct sr_channel *)l->data)->enabled
					&& !strcmp(((struct sr_channel *)l->data)->name, tokens[i]))
				ch = l->data;
		if (!ch)
			g_critical("Invalid channel '%s'.", tokens[i]);
		for (int t = 0; sep[t]; t++) {
			match = 0;
			switch (sep[t]) {
			case '0': match = SR_TRIGGER_ZERO; break;
			case '1': match = SR_TRIGGER_ONE; break;
			case 'r': match = SR_TRIGGER_RISING; break;
			case 'f': match = SR_TRIGGER_FALLING; break;
			case 'e': match = SR_TRIGGER_EDGE; break;
			case 'o': match = SR_TRIGGER_OVER; break;
			case 'u': match = SR_TRIGGER_UNDER; break;
			}
			if (!match)
				g_critical("Invalid trigger match '%c'.", sep[t]);
			found = FALSE;
			for (gsize j = 0; j < num_matches; j++)
				found |= matches[j] == match;
			if (!found)
				g_critical("Trigger match '%c' not supported by device.", sep[t]);
			while (!(stage = g_slist_nth_data(trigger->stages, t)))
				sr_trigger_stage_add(trigger);
			/* libsigrok has logged why; sigrok-cli adds nothing
			 * and gives up without a failing status. */
			if (sr_trigger_match_add(stage, ch, match, 0) != SR_OK) {
				sr_trigger_free(trigger);
				trigger = NULL;
				goto done;
			}
		}
	}
done:
	g_strfreev(tokens);
	g_variant_unref(gvar);
	return trigger;
}

/* session.c's setup_output_format(), to stdout. */
static const struct sr_output *setup_output_format(const struct sr_dev_inst *sdi)
{
	const struct sr_output_module *omod;
	const struct sr_option **options;
	GHashTable *fmtargs, *fmtopts = NULL;
	GVariant *gvar = NULL;
	GHashTableIter iter;
	gpointer key;
	const char *fmtspec, *s;
	const struct sr_output *o;
	int i;

	fmtargs = parse_generic_arg(opt_output_format ? opt_output_format : "bits", TRUE);
	fmtspec = g_hash_table_lookup(fmtargs, "sigrok_key");
	if (!(omod = sr_output_find((char *)fmtspec)))
		g_critical("Unknown output module '%s'.", fmtspec);
	if ((options = sr_output_options_get(omod))) {
		fmtopts = g_hash_table_new_full(g_str_hash, g_str_equal, g_free,
			(GDestroyNotify)g_variant_unref);
		/* parsers.c's warn_unknown_keys(). */
		g_hash_table_iter_init(&iter, fmtargs);
		while (g_hash_table_iter_next(&iter, &key, NULL)) {
			if (!strcmp(key, "sigrok_key"))
				continue;
			for (i = 0; options[i]; i++)
				if (!strcmp(options[i]->id, key))
					break;
			if (!options[i])
				g_warning("Unknown keyword: %s.", (char *)key);
		}
		for (i = 0; options[i]; i++) {
			if (!(s = g_hash_table_lookup(fmtargs, options[i]->id)))
				continue;
			if (g_variant_is_of_type(options[i]->def, G_VARIANT_TYPE_UINT32))
				gvar = g_variant_new_uint32(strtoul(s, NULL, 10));
			else if (g_variant_is_of_type(options[i]->def, G_VARIANT_TYPE_INT32))
				gvar = g_variant_new_int32(strtol(s, NULL, 10));
			else if (g_variant_is_of_type(options[i]->def, G_VARIANT_TYPE_UINT64))
				gvar = g_variant_new_uint64(strtoull(s, NULL, 10));
			else if (g_variant_is_of_type(options[i]->def, G_VARIANT_TYPE_DOUBLE))
				gvar = g_variant_new_double(strtod(s, NULL));
			else if (g_variant_is_of_type(options[i]->def, G_VARIANT_TYPE_BOOLEAN)) {
				if (strcmp(s, "true") && strcmp(s, "yes") && strcmp(s, "false") && strcmp(s, "no"))
					g_critical("Unable to convert '%s' to boolean!", s);
				gvar = g_variant_new_boolean(strcmp(s, "false") && strcmp(s, "no"));
			} else if (g_variant_is_of_type(options[i]->def, G_VARIANT_TYPE_STRING))
				gvar = g_variant_new_string(s);
			else
				g_critical("Don't know GVariant type for option '%s'!", options[i]->id);
			g_hash_table_insert(fmtopts, g_strdup(options[i]->id), g_variant_ref_sink(gvar));
		}
		sr_output_options_free(options);
	}
	o = sr_output_new(omod, fmtopts, sdi, NULL);
	if (fmtopts)
		g_hash_table_destroy(fmtopts);
	g_hash_table_destroy(fmtargs);
	return o;
}

/* The stop --stop-after asks for, out of the datafeed callback. */
static gboolean stop_idle(gpointer data)
{
	(void)data;
	sr_session_stop(session);
	return G_SOURCE_REMOVE;
}

static void request_stop(void)
{
	static gboolean requested;

	if (!requested) {
		requested = TRUE;
		g_idle_add(stop_idle, NULL);
	}
}

/* session.c's datafeed_in() without decoders, plus --stop-after. */
static void datafeed_in(const struct sr_dev_inst *sdi,
		const struct sr_datafeed_packet *packet, void *cb_data)
{
	static const struct sr_output *o, *oa;
	static uint64_t rcvd_logic, rcvd_analog;
	const struct sr_datafeed_packet *send = packet;
	struct sr_datafeed_packet cut;
	struct sr_datafeed_logic logic_cut;
	struct sr_datafeed_analog analog_cut;
	const struct sr_datafeed_logic *logic;
	const struct sr_datafeed_analog *analog;
	uint64_t n;
	GString *out;

	(void)cb_data;
	if (packet->type != SR_DF_HEADER && !o)
		return;
	switch (packet->type) {
	case SR_DF_HEADER:
		if (!(o = setup_output_format(sdi)))
			g_critical("Failed to initialize output module.");
		oa = sr_output_new(sr_output_find("analog"), NULL, sdi, NULL);
		rcvd_logic = rcvd_analog = 0;
		if (have_stop && stop_after == 0)
			request_stop();
		break;
	case SR_DF_LOGIC:
		logic = packet->payload;
		if (logic->length == 0)
			break;
		n = logic->length / logic->unitsize;
		if (have_stop) {
			if (rcvd_logic >= stop_after)
				return;
			if (rcvd_logic + n > stop_after) {
				n = stop_after - rcvd_logic;
				logic_cut = *logic;
				logic_cut.length = n * logic->unitsize;
				cut = *packet;
				cut.payload = &logic_cut;
				send = &cut;
			}
		}
		rcvd_logic += n;
		break;
	case SR_DF_ANALOG:
		analog = packet->payload;
		if (analog->num_samples == 0)
			break;
		/* With logic channels too, the logic ones count. */
		if (have_stop && rcvd_logic == 0) {
			if (rcvd_analog >= stop_after)
				return;
			if (rcvd_analog + analog->num_samples > stop_after) {
				analog_cut = *analog;
				analog_cut.num_samples = stop_after - rcvd_analog;
				cut = *packet;
				cut.payload = &analog_cut;
				send = &cut;
			}
		}
		rcvd_analog += ((const struct sr_datafeed_analog *)send->payload)->num_samples;
		break;
	default:
		break;
	}
	if (sr_output_send(o, send, &out) == SR_OK) {
		if (oa && !out)
			sr_output_send(oa, send, &out);
		if (out && out->len > 0) {
			fwrite(out->str, 1, out->len, stdout);
			fflush(stdout);
		}
		if (out)
			g_string_free(out, TRUE);
	}
	if (have_stop && (packet->type == SR_DF_LOGIC || packet->type == SR_DF_ANALOG)
			&& (rcvd_logic >= stop_after || (rcvd_logic == 0 && rcvd_analog >= stop_after)))
		request_stop();
	if (packet->type == SR_DF_END) {
		sr_output_free(o);
		o = NULL;
		if (oa)
			sr_output_free(oa);
		oa = NULL;
		n = limit_samples ? limit_samples : stop_after;
		if (n && rcvd_logic > 0 && rcvd_logic < n)
			g_warning("Device only sent %" PRIu64 " samples.", rcvd_logic);
		else if (n && rcvd_analog > 0 && rcvd_analog < n)
			g_warning("Device only sent %" PRIu64 " samples.", rcvd_analog);
	}
}

/* session.c's set_dev_options_array(), for the device as a whole. */
static void set_dev_options(struct sr_dev_inst *sdi)
{
	GHashTableIter iter;
	gpointer k, v;
	struct sr_config src;
	int ret;

	for (guint i = 0; opt_configs && i < opt_configs->len; i++) {
		GHashTable *args = parse_generic_arg(g_ptr_array_index(opt_configs, i), FALSE);

		if (!args)
			continue;
		g_hash_table_iter_init(&iter, args);
		while (g_hash_table_iter_next(&iter, &k, &v)) {
			if (opt_to_gvar(k, v, &src) != 0)
				exit(1);
			if ((ret = maybe_config_set(sdi, src.key, src.data)) != SR_OK)
				g_critical("Failed to set device option '%s': %s.", (char *)k, sr_strerror(ret));
		}
		g_hash_table_destroy(args);
	}
}

/* session.c's set_limit_time(). */
static void set_limit_time(const struct sr_dev_inst *sdi)
{
	uint64_t msec, samplerate;
	GVariant *gvar;

	if (!(msec = sr_parse_timestring(opt_time)))
		g_critical("Invalid time '%s'", opt_time);
	if (sr_dev_config_capabilities_list(sdi, NULL, SR_CONF_LIMIT_MSEC) & SR_CONF_SET) {
		if (sr_config_set(sdi, NULL, SR_CONF_LIMIT_MSEC, g_variant_new_uint64(msec)) != SR_OK)
			g_critical("Failed to configure time limit.");
	} else if (sr_dev_config_capabilities_list(sdi, NULL, SR_CONF_SAMPLERATE)
			& (SR_CONF_GET | SR_CONF_SET)) {
		sr_config_get(sr_dev_inst_driver_get(sdi), sdi, NULL, SR_CONF_SAMPLERATE, &gvar);
		samplerate = g_variant_get_uint64(gvar);
		g_variant_unref(gvar);
		limit_samples = samplerate * msec / 1000;
		if (limit_samples == 0)
			g_critical("Not enough time at this samplerate.");
		if (sr_config_set(sdi, NULL, SR_CONF_LIMIT_SAMPLES, g_variant_new_uint64(limit_samples)) != SR_OK)
			g_critical("Failed to configure time-based sample limit.");
	} else {
		g_critical("This device does not support time limits.");
	}
}

static void run_session(void)
{
	GSList *devices;
	struct sr_dev_inst *sdi;
	struct sr_trigger *trigger = NULL;
	uint64_t frames, min_samples, max_samples;
	GVariant *gvar;
	GMainLoop *main_loop;

	if (!(devices = device_scan()))
		g_critical("No devices found.");
	if (devices->next)
		g_critical("libsigrok-run only supports one device for capturing.");
	sdi = devices->data;
	g_slist_free(devices);

	sr_session_new(sr_ctx, &session);
	sr_session_datafeed_callback_add(session, datafeed_in, NULL);
	if (sr_dev_open(sdi) != SR_OK)
		g_critical("Failed to open device.");
	if (sr_session_dev_add(session, sdi) != SR_OK)
		g_critical("Failed to add device to session.");
	set_dev_options(sdi);
	select_channels(sdi);
	if (opt_triggers) {
		trigger = parse_triggerstring(sdi, opt_triggers);
		if (!trigger || sr_session_trigger_set(session, trigger) != SR_OK) {
			sr_session_destroy(session);
			return;
		}
	}
	if (opt_time)
		set_limit_time(sdi);
	if (opt_samples) {
		if (sr_parse_sizestring(opt_samples, &limit_samples) != SR_OK)
			g_critical("Invalid sample limit '%s'.", opt_samples);
		if (maybe_config_list(sdi, SR_CONF_LIMIT_SAMPLES, &gvar) == SR_OK) {
			g_variant_get(gvar, "(tt)", &min_samples, &max_samples);
			g_variant_unref(gvar);
			if (limit_samples < min_samples)
				g_critical("The device stores at least %" PRIu64
					" samples with the current settings.", min_samples);
			if (limit_samples > max_samples)
				g_critical("The device can store only %" PRIu64
					" samples with the current settings.", max_samples);
		}
		if (maybe_config_set(sdi, SR_CONF_LIMIT_SAMPLES, g_variant_new_uint64(limit_samples)) != SR_OK)
			g_critical("Failed to configure sample limit.");
	}
	if (opt_frames) {
		if (sr_parse_sizestring(opt_frames, &frames) != SR_OK)
			g_critical("Invalid frame limit '%s'.", opt_frames);
		if (maybe_config_set(sdi, SR_CONF_LIMIT_FRAMES, g_variant_new_uint64(frames)) != SR_OK)
			g_critical("Failed to configure frame limit.");
	}
	if (opt_stop_after) {
		if (sr_parse_sizestring(opt_stop_after, &stop_after) != SR_OK)
			g_critical("Invalid --stop-after '%s'.", opt_stop_after);
		have_stop = TRUE;
	}

	main_loop = g_main_loop_new(NULL, FALSE);
	sr_session_stopped_callback_set(session, (sr_session_stopped_callback)g_main_loop_quit, main_loop);
	if (sr_session_start(session) != SR_OK)
		g_critical("Failed to start session.");
	g_main_loop_run(main_loop);
	if (trigger)
		sr_trigger_free(trigger);
	sr_session_datafeed_callback_remove_all(session);
	g_main_loop_unref(main_loop);
	sr_session_destroy(session);
}

static void usage(void)
{
	fprintf(stderr, "usage: libsigrok-run -d DRIVER[:KEY=VALUE...] [-l LEVEL] [-c KEY=VALUE[:...]]...\n"
		"         [-C CHANNELS] [-t TRIGGER] [-O FORMAT[:KEY=VALUE...]]\n"
		"         [--samples N] [--time T] [--frames N] [--stop-after N] | --scan\n");
	exit(2);
}

int main(int argc, char **argv)
{
	g_log_set_default_handler(logger, NULL);
	opt_configs = g_ptr_array_new();
	for (int i = 1; i < argc; i++) {
		const char *a = argv[i];
		char **val = NULL;

		if (!strcmp(a, "--scan")) {
			opt_scan = TRUE;
			continue;
		}
		if (i + 1 >= argc)
			usage();
		if (!strcmp(a, "-d"))
			val = &opt_drv;
		else if (!strcmp(a, "-C"))
			val = &opt_channels;
		else if (!strcmp(a, "-t") || !strcmp(a, "--triggers"))
			val = &opt_triggers;
		else if (!strcmp(a, "-O"))
			val = &opt_output_format;
		else if (!strcmp(a, "--samples"))
			val = &opt_samples;
		else if (!strcmp(a, "--time"))
			val = &opt_time;
		else if (!strcmp(a, "--frames"))
			val = &opt_frames;
		else if (!strcmp(a, "--stop-after"))
			val = &opt_stop_after;
		else if (!strcmp(a, "-c"))
			g_ptr_array_add(opt_configs, argv[i + 1]);
		else if (!strcmp(a, "-l"))
			opt_loglevel = atoi(argv[i + 1]);
		else
			usage();
		if (val)
			*val = argv[i + 1];
		i++;
	}
	if (!opt_drv)
		usage();
	if (sr_log_loglevel_set(opt_loglevel) != SR_OK || sr_init(&sr_ctx) != SR_OK)
		return 1;
	if (opt_scan)
		show_dev_list();
	else
		run_session();
	sr_exit(sr_ctx);
	return 0;
}
