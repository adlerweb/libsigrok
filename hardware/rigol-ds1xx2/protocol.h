/*
 * This file is part of the libsigrok project.
 *
 * Copyright (C) 2012 Martin Ling <martin-git@earth.li>
 * Copyright (C) 2013 Bert Vermeulen <bert@biot.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef LIBSIGROK_HARDWARE_RIGOL_DS1XX2_PROTOCOL_H
#define LIBSIGROK_HARDWARE_RIGOL_DS1XX2_PROTOCOL_H

#include <stdint.h>
#include "libsigrok.h"
#include "libsigrok-internal.h"

/* Message logging helpers with driver-specific prefix string. */
#define DRIVER_LOG_DOMAIN "rigol-ds1xx2: "
#define sr_log(l, s, args...) sr_log(l, DRIVER_LOG_DOMAIN s, ## args)
#define sr_spew(s, args...) sr_spew(DRIVER_LOG_DOMAIN s, ## args)
#define sr_dbg(s, args...) sr_dbg(DRIVER_LOG_DOMAIN s, ## args)
#define sr_info(s, args...) sr_info(DRIVER_LOG_DOMAIN s, ## args)
#define sr_warn(s, args...) sr_warn(DRIVER_LOG_DOMAIN s, ## args)
#define sr_err(s, args...) sr_err(DRIVER_LOG_DOMAIN s, ## args)

#define WAVEFORM_SIZE 600

/** Private, per-device-instance driver context. */
struct dev_context {
	/* Acquisition settings */
	GSList *enabled_probes;
	uint64_t limit_frames;
	void *cb_data;

	/* Device settings */
	gboolean channels[2];
	float timebase;
	float vdiv[2];
	float vert_offset[2];
	char *trigger_source;
	float horiz_triggerpos;
	char *trigger_slope;
	char *coupling[2];

	/* Operational state */
	char *device;
	int fd;
	uint64_t num_frames;
	uint64_t num_frame_samples;
	struct sr_probe *channel_frame;
};

SR_PRIV int rigol_ds1xx2_receive(int fd, int revents, void *cb_data);
SR_PRIV int rigol_ds1xx2_send(struct dev_context *devc, const char *format, ...);
SR_PRIV int rigol_ds1xx2_get_dev_cfg(const struct sr_dev_inst *sdi);

#endif
