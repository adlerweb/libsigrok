/*
 * This file is part of the sigrok project.
 *
 * Copyright (C) 2012 Uwe Hermann <uwe@hermann-uwe.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 USA
 */

#include <stdlib.h>
#include <string.h>
#include "libsigrok.h"
#include "libsigrok-internal.h"
#include "protocol.h"

static const int hwcaps[] = {
	SR_HWCAP_MULTIMETER,
	SR_HWCAP_LIMIT_SAMPLES,
	SR_HWCAP_LIMIT_MSEC,
	SR_HWCAP_CONTINUOUS,
	0,
};

static const char *probe_names[] = {
	"Probe",
	NULL,
};

SR_PRIV struct sr_dev_driver uni_t_dmm_driver_info;
static struct sr_dev_driver *di = &uni_t_dmm_driver_info;

static int open_usb(struct sr_dev_inst *sdi)
{
	libusb_device **devlist;
	struct libusb_device_descriptor des;
	struct dev_context *devc;
	int ret, tmp, cnt, i;

	/* TODO: Use common code later, refactor. */

	devc = sdi->priv;

	if ((cnt = libusb_get_device_list(NULL, &devlist)) < 0) {
		sr_err("Error getting USB device list: %d.", cnt);
		return SR_ERR;
	}

	ret = SR_ERR;
	for (i = 0; i < cnt; i++) {
		if ((tmp = libusb_get_device_descriptor(devlist[i], &des))) {
			sr_err("Failed to get device descriptor: %d.", tmp);
			continue;
		}

		if (libusb_get_bus_number(devlist[i]) != devc->usb->bus
			|| libusb_get_device_address(devlist[i]) != devc->usb->address)
			continue;

		if ((tmp = libusb_open(devlist[i], &devc->usb->devhdl))) {
			sr_err("Failed to open device: %d.", tmp);
			break;
		}

		sr_info("Opened USB device on %d.%d.",
			devc->usb->bus, devc->usb->address);
		ret = SR_OK;
		break;
	}
	libusb_free_device_list(devlist, 1);

	return ret;
}

static GSList *connect_usb(const char *conn)
{
	struct sr_dev_inst *sdi;
	struct drv_context *drvc;
	struct dev_context *devc;
	struct sr_probe *probe;
	libusb_device **devlist;
	struct libusb_device_descriptor des;
	GSList *devices;
	int vid, pid, devcnt, err, i;

	(void)conn;

	/* TODO: Use common code later, refactor. */

	drvc = di->priv;

	/* Hardcoded for now. */
	vid = UT_D04_CABLE_USB_VID;
	pid = UT_D04_CABLE_USB_DID;

	devices = NULL;
	libusb_get_device_list(NULL, &devlist);
	for (i = 0; devlist[i]; i++) {
		if ((err = libusb_get_device_descriptor(devlist[i], &des))) {
			sr_err("Failed to get device descriptor: %d", err);
			continue;
		}

		if (des.idVendor != vid || des.idProduct != pid)
			continue;

		if (!(devc = g_try_malloc0(sizeof(struct dev_context)))) {
			sr_err("Device context malloc failed.");
			return NULL;
		}

		devcnt = g_slist_length(drvc->instances);
		if (!(sdi = sr_dev_inst_new(devcnt, SR_ST_INACTIVE,
					    "UNI-T DMM", NULL, NULL))) {
			sr_err("sr_dev_inst_new returned NULL.");
			return NULL;
		}
		sdi->priv = devc;
		if (!(probe = sr_probe_new(0, SR_PROBE_ANALOG, TRUE, "P1")))
			return NULL;
		sdi->probes = g_slist_append(sdi->probes, probe);
		devc->usb = sr_usb_dev_inst_new(
				libusb_get_bus_number(devlist[i]),
				libusb_get_device_address(devlist[i]), NULL);
		devices = g_slist_append(devices, sdi);
	}
	libusb_free_device_list(devlist, 1);

	return devices;
}

static int clear_instances(void)
{
	/* TODO: Use common code later. */

	return SR_OK;
}

static int hw_init(void)
{
	int ret;
	struct drv_context *drvc;

	if (!(drvc = g_try_malloc0(sizeof(struct drv_context)))) {
		sr_err("Driver context malloc failed.");
		return SR_ERR_MALLOC;
	}

	if ((ret = libusb_init(NULL)) < 0) {
		sr_err("Failed to initialize libusb: %s.",
		       libusb_error_name(ret));
		return SR_ERR;
	}

	di->priv = drvc;

	return SR_OK;
}

static GSList *hw_scan(GSList *options)
{
	GSList *l, *devices;
	struct sr_dev_inst *sdi;
	struct drv_context *drvc;

	(void)options;

	drvc = di->priv;

	if (!(devices = connect_usb(NULL)))
		return NULL;

	for (l = devices; l; l = l->next) {
		sdi = l->data;
		sdi->driver = di;
		drvc->instances = g_slist_append(drvc->instances, l->data);
	}

	return devices;
}

static GSList *hw_dev_list(void)
{
	struct drv_context *drvc;

	drvc = di->priv;

	return drvc->instances;
}

static int hw_dev_open(struct sr_dev_inst *sdi)
{
	return open_usb(sdi);
}

static int hw_dev_close(struct sr_dev_inst *sdi)
{
	(void)sdi;

	/* TODO */

	return SR_OK;
}

static int hw_cleanup(void)
{
	clear_instances();

	// libusb_exit(NULL);

	return SR_OK;
}

static int hw_info_get(int info_id, const void **data,
		       const struct sr_dev_inst *sdi)
{
	(void)sdi;

	sr_spew("Backend requested info_id %d.", info_id);

	switch (info_id) {
	case SR_DI_HWCAPS:
		*data = hwcaps;
		sr_spew("%s: Returning hwcaps.", __func__);
		break;
	case SR_DI_NUM_PROBES:
		*data = GINT_TO_POINTER(1);
		sr_spew("%s: Returning number of probes.", __func__);
		break;
	case SR_DI_PROBE_NAMES:
		*data = probe_names;
		sr_spew("%s: Returning probe names.", __func__);
		break;
	case SR_DI_SAMPLERATES:
		/* TODO: Get rid of this. */
		*data = NULL;
		sr_spew("%s: Returning samplerates.", __func__);
		return SR_ERR_ARG;
		break;
	case SR_DI_CUR_SAMPLERATE:
		/* TODO: Get rid of this. */
		*data = NULL;
		sr_spew("%s: Returning current samplerate.", __func__);
		return SR_ERR_ARG;
		break;
	default:
		sr_err("%s: Unknown info_id %d.", __func__, info_id);
		return SR_ERR_ARG;
		break;
	}

	return SR_OK;
}

static int hw_dev_config_set(const struct sr_dev_inst *sdi, int hwcap,
			     const void *value)
{
	struct dev_context *devc;

	devc = sdi->priv;

	switch (hwcap) {
	case SR_HWCAP_LIMIT_MSEC:
		/* TODO: Not yet implemented. */
		if (*(const uint64_t *)value == 0) {
			sr_err("Time limit cannot be 0.");
			return SR_ERR;
		}
		devc->limit_msec = *(const uint64_t *)value;
		sr_dbg("Setting time limit to %" PRIu64 "ms.",
		       devc->limit_msec);
		break;
	case SR_HWCAP_LIMIT_SAMPLES:
		if (*(const uint64_t *)value == 0) {
			sr_err("Sample limit cannot be 0.");
			return SR_ERR;
		}
		devc->limit_samples = *(const uint64_t *)value;
		sr_dbg("Setting sample limit to %" PRIu64 ".",
		       devc->limit_samples);
		break;
	default:
		sr_err("Unknown capability: %d.", hwcap);
		return SR_ERR;
		break;
	}

	return SR_OK;
}

static int hw_dev_acquisition_start(const struct sr_dev_inst *sdi,
				    void *cb_data)
{
	struct sr_datafeed_packet packet;
	struct sr_datafeed_header header;
	struct sr_datafeed_meta_analog meta;
	struct dev_context *devc;

	devc = sdi->priv;

	sr_dbg("Starting acquisition.");

	devc->cb_data = cb_data;

	/* Send header packet to the session bus. */
	sr_dbg("Sending SR_DF_HEADER.");
	packet.type = SR_DF_HEADER;
	packet.payload = (uint8_t *)&header;
	header.feed_version = 1;
	gettimeofday(&header.starttime, NULL);
	sr_session_send(devc->cb_data, &packet);

	/* Send metadata about the SR_DF_ANALOG packets to come. */
	sr_dbg("Sending SR_DF_META_ANALOG.");
	packet.type = SR_DF_META_ANALOG;
	packet.payload = &meta;
	meta.num_probes = 1;
	sr_session_send(devc->cb_data, &packet);

	sr_source_add(0, 0, 10 /* poll_timeout */,
		      uni_t_dmm_receive_data, (void *)sdi);

	return SR_OK;
}

static int hw_dev_acquisition_stop(const struct sr_dev_inst *sdi,
				   void *cb_data)
{
	struct sr_datafeed_packet packet;

	(void)sdi;

	sr_dbg("Stopping acquisition.");

	/* Send end packet to the session bus. */
	sr_dbg("Sending SR_DF_END.");
	packet.type = SR_DF_END;
	sr_session_send(cb_data, &packet);

	/* TODO? */
	sr_source_remove(0);

	return SR_OK;
}

SR_PRIV struct sr_dev_driver uni_t_dmm_driver_info = {
	.name = "uni-t-dmm",
	.longname = "UNI-T DMM series",
	.api_version = 1,
	.init = hw_init,
	.cleanup = hw_cleanup,
	.scan = hw_scan,
	.dev_list = hw_dev_list,
	.dev_clear = clear_instances,
	.dev_open = hw_dev_open,
	.dev_close = hw_dev_close,
	.info_get = hw_info_get,
	.dev_config_set = hw_dev_config_set,
	.dev_acquisition_start = hw_dev_acquisition_start,
	.dev_acquisition_stop = hw_dev_acquisition_stop,
	.priv = NULL,
};