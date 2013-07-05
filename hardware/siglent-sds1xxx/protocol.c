/*
 * This file is part of the libsigrok project.
 *
 * Copyright (C) 2012 Martin Ling <martin-git@earth.li>
 * Copyright (C) 2013 Bert Vermeulen <bert@biot.com>
 * Copyright (C) 2013 Florian Knodt <adlerweb@adlerweb.info>
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

#include <stdlib.h>
#include <stdarg.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <math.h>
#include <glib.h>
#include "libsigrok.h"
#include "libsigrok-internal.h"
#include "protocol.h"

SR_PRIV int siglent_sds1xxx_receive(int fd, int revents, void *cb_data)
{
	struct sr_dev_inst *sdi;
	struct sr_serial_dev_inst *serial;
	struct dev_context *devc;
    unsigned char buf[ANALOG_WAVEFORM_SIZE];
	struct sr_datafeed_packet packet;
	struct sr_datafeed_analog analog;
	double vdiv, offset;
	float data[ANALOG_WAVEFORM_SIZE];
	int len, i, waveform_size;
	struct sr_probe *probe;
    
	(void)fd;

	if (!(sdi = cb_data))
		return TRUE;

	if (!(devc = sdi->priv))
		return TRUE;

	serial = sdi->conn;

	if (revents == G_IO_IN) {
		probe = devc->channel_frame;
		waveform_size = ANALOG_WAVEFORM_SIZE;
		len = serial_read(serial, buf, waveform_size - devc->num_frame_bytes);
		sr_dbg("Received %d bytes.", len);

		if (first_frame == TRUE) {
			/* Start of a new frame. */
			packet.type = SR_DF_FRAME_BEGIN;
			sr_session_send(sdi, &packet);
            first_frame = FALSE;
		}

        if(len == -1) {
		    packet.type = SR_DF_FRAME_END;
		    sr_session_send(sdi, &packet);
		    devc->num_frame_bytes = 0;
            return TRUE;
        }

		for (i = 0; i < len; i++) {
			vdiv = devc->vdiv[probe->index];
			offset = devc->vert_offset[probe->index];
			//data[i] = vdiv / 25.6 * (128 - buf[i]) - offset;
            //sr_dbg("Raw Data was: %3.9f computed value %3.9f with division %3.9f and offset %3.9f", buf[i], data[i], vdiv, offset);
            data[i] = buf[i] - offset;
		}
		analog.probes = g_slist_append(NULL, probe);
		analog.num_samples = len;
		analog.data = data;
		analog.mq = SR_MQ_VOLTAGE;
		analog.unit = SR_UNIT_VOLT;
		analog.mqflags = 0;
		packet.type = SR_DF_ANALOG;
		packet.payload = &analog;
		sr_session_send(cb_data, &packet);
		g_slist_free(analog.probes);

		if (len == ANALOG_WAVEFORM_SIZE)
			/* most likely a new frame will follow... */
			return TRUE;
		
		/* End of the frame. */
		packet.type = SR_DF_FRAME_END;
		sr_session_send(sdi, &packet);
		devc->num_frame_bytes = 0;

		if (devc->enabled_analog_probes
				&& devc->channel_frame == devc->enabled_analog_probes->data
				&& devc->enabled_analog_probes->next != NULL &&
                devc->analog_channels[devc->channel_frame->index]) {
			/* We got the frame for the first analog channel, but
			 * there's a second analog channel. */
			devc->channel_frame = devc->enabled_analog_probes->next->data;
			siglent_sds1xxx_send(sdi, "C%d:WF?",
					devc->channel_frame->index + 1);
            first_frame = TRUE;
		} else {
            if (++devc->num_frames == devc->limit_frames) {
                /* End of last frame. */
                sdi->driver->dev_acquisition_stop(sdi, cb_data);
            } else {
                /* Get the next frame, starting with the first analog channel. */
                devc->channel_frame = devc->enabled_analog_probes->data;
                rigol_ds1xx2_send(sdi, "C%d:WF?",
                        devc->channel_frame->index);
            }
        }
	}

	return TRUE;
}

SR_PRIV int siglent_sds1xxx_send(const struct sr_dev_inst *sdi, const char *format, ...)
{
	va_list args;
	char buf[256];
	int len, out, ret;

	va_start(args, format);
	len = vsnprintf(buf, 255, format, args);
	va_end(args);
	strcat(buf, "\n");
	len++;
	out = serial_write(sdi->conn, buf, len);
	buf[len - 1] = '\0';
	if (out != len) {
		sr_dbg("Only sent %d/%d bytes of '%s'.", out, len, buf);
		ret = SR_ERR;
	} else {
		sr_spew("Sent '%s'.", buf);
		ret = SR_OK;
	}

	return ret;
}

static int get_cfg(const struct sr_dev_inst *sdi, char *cmd, char *reply)
{
	int len;

	if (siglent_sds1xxx_send(sdi, cmd) != SR_OK)
		return SR_ERR;

	if ((len = serial_read(sdi->conn, reply, 255)) < 0)
		return SR_ERR;
	reply[len] = '\0';
	sr_spew("Received '%s'.", reply);

	return SR_OK;
}

static int get_cfg_float(const struct sr_dev_inst *sdi, char *cmd, float *f)
{
	char buf[256], *e;

	if (get_cfg(sdi, cmd, buf) != SR_OK)
		return SR_ERR;
	*f = strtof(buf, &e);
	if (e == buf || (fpclassify(*f) & (FP_ZERO | FP_NORMAL)) == 0) {
		sr_dbg("failed to parse response to '%s': '%s'", cmd, buf);
		return SR_ERR;
	}

	return SR_OK;
}

static int get_cfg_string(const struct sr_dev_inst *sdi, char *cmd, char **buf)
{

	if (!(*buf = g_try_malloc0(256)))
		return SR_ERR_MALLOC;

	if (get_cfg(sdi, cmd, *buf) != SR_OK)
		return SR_ERR;

	return SR_OK;
}

SR_PRIV int siglent_sds1xxx_get_dev_cfg(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc;
    char *buf;
    gchar **tokens;
	char *t_s;

	devc = sdi->priv;

	/* Analog channel state. */
	if (get_cfg_string(sdi, "C1:TRA?", &t_s) != SR_OK)
		return SR_ERR;
	devc->analog_channels[0] = !strcmp(t_s, "ON\n") ? TRUE : FALSE;
	g_free(t_s);
	if (get_cfg_string(sdi, "C2:TRA?", &t_s) != SR_OK)
		return SR_ERR;
	devc->analog_channels[1] = !strcmp(t_s, "ON\n") ? TRUE : FALSE;
	g_free(t_s);
	sr_dbg("Current analog channel state C1 %s C2 %s",
			devc->analog_channels[0] ? "on" : "off",
			devc->analog_channels[1] ? "on" : "off");

	/* Timebase. */
	if (get_cfg_float(sdi, ":TDIV?", &devc->timebase) != SR_OK)
		return SR_ERR;
	sr_dbg("Current timebase %f", devc->timebase);

	/* Vertical gain. */
	if (get_cfg_float(sdi, "C1:VDIV?", &devc->vdiv[0]) != SR_OK)
		return SR_ERR;
	if (get_cfg_float(sdi, "C2:VDIV?", &devc->vdiv[1]) != SR_OK)
		return SR_ERR;
	sr_dbg("Current vertical gain C1 %f C2 %f", devc->vdiv[0], devc->vdiv[1]);

	/* Vertical offset. */
	if (get_cfg_float(sdi, "C1:OFST?", &devc->vert_offset[0]) != SR_OK)
		return SR_ERR;
	if (get_cfg_float(sdi, "C2:OFST?", &devc->vert_offset[1]) != SR_OK)
		return SR_ERR;
	sr_dbg("Current vertical offset C1 %f C2 %f", devc->vert_offset[0],
			devc->vert_offset[1]);

	/* Coupling. */
	if (get_cfg_string(sdi, "C1:CPL?", &devc->coupling[0]) != SR_OK)
		return SR_ERR;
	if (get_cfg_string(sdi, "C2:CPL?", &devc->coupling[1]) != SR_OK)
		return SR_ERR;
    devc->coupling[0][ strlen(devc->coupling[0]) - 1 ] = '\0'; //Drop newline
    devc->coupling[1][ strlen(devc->coupling[1]) - 1 ] = '\0'; //Drop newline
	sr_dbg("Current coupling C1 %s C2 %s", devc->coupling[0],
			devc->coupling[1]);

	/* Trigger source. */
	if (get_cfg_string(sdi, ":TRSE?", &buf) != SR_OK)
		return SR_ERR;
    tokens = g_strsplit(buf, ",", 0);
    devc->trigger_source = tokens[2];
	sr_dbg("Current trigger source %s", devc->trigger_source);

	/* Horizontal trigger position. */
    if(strcmp(devc->trigger_source, "C1")) {
    	if (get_cfg_float(sdi, "C1:TRLV?", &devc->horiz_triggerpos) != SR_OK)
	    	return SR_ERR;
    } else if(strcmp(devc->trigger_source, "C2")) {
    	if (get_cfg_float(sdi, "C2:TRLV?", &devc->horiz_triggerpos) != SR_OK)
	    	return SR_ERR;
    } else {
        sr_dbg("Failed to read horizontal trigger position - trigger source error");
        return SR_ERR;
    }
	sr_dbg("Current horizontal trigger position %f", devc->horiz_triggerpos);

	/* Trigger slope. */
	if(strcmp(devc->trigger_source, "C1")) {
        if (get_cfg_string(sdi, "C1:TRSL?", &devc->trigger_slope) != SR_OK)
		    return SR_ERR;
    } else if(strcmp(devc->trigger_source, "C2")) {
        if (get_cfg_string(sdi, "C2:TRSL?", &devc->trigger_slope) != SR_OK)
            return SR_ERR;
    } else{
        sr_dbg("Failed to read trigger slope - trigger source error");
            return SR_ERR;
    }
	sr_dbg("Current trigger slope %s", devc->trigger_slope);

	return SR_OK;
}

