// Display calibration backend for DRM/KMS platforms, uploads the LUT to the
// internal panel through the CRTC GAMMA_LUT property.
#include "displaycal.h"

#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <xf86drm.h>
#include <xf86drmMode.h>

#define DISPLAYCAL_CARD_COUNT 4

typedef struct {
	double red_gain;
	double green_gain;
	double blue_gain;
} DisplayCalGains;

typedef struct {
	int fd;
	uint32_t crtc_id;
	uint32_t lut_prop;
	uint32_t lut_size;
} DisplayCalTarget;

static uint16_t clamp_u16(double value) {
	if (value < 0.0)
		return 0;
	if (value > 65535.0)
		return 65535;
	return (uint16_t)(value + 0.5);
}

static double srgb_to_linear(double c) {
	if (c <= 0.04045)
		return c / 12.92;
	return pow((c + 0.055) / 1.055, 2.4);
}

static double linear_to_srgb(double c) {
	if (c <= 0.0)
		return 0.0;
	if (c <= 0.0031308)
		return c * 12.92;
	return 1.055 * pow(c, 1.0 / 2.4) - 0.055;
}

static uint16_t gain_entry(double linear, double gain) {
	return clamp_u16(linear_to_srgb(linear * gain) * 65535.0);
}

static void fill_linear_gain_table(struct drm_color_lut *table, uint32_t entries, const DisplayCalGains *gains) {
	for (uint32_t i = 0; i < entries; i++) {
		double linear = srgb_to_linear((double)i / (entries - 1));
		table[i].red = gain_entry(linear, gains->red_gain);
		table[i].green = gain_entry(linear, gains->green_gain);
		table[i].blue = gain_entry(linear, gains->blue_gain);
	}
}

static int find_crtc_prop(int fd, uint32_t crtc_id, const char *name, uint32_t *prop_id, uint64_t *value) {
	drmModeObjectProperties *props = drmModeObjectGetProperties(fd, crtc_id, DRM_MODE_OBJECT_CRTC);
	if (!props)
		return -1;

	int found = -1;
	for (uint32_t i = 0; i < props->count_props && found; i++) {
		drmModePropertyRes *prop = drmModeGetProperty(fd, props->props[i]);
		if (!prop)
			continue;
		if (strcmp(prop->name, name) == 0) {
			if (prop_id)
				*prop_id = prop->prop_id;
			if (value)
				*value = props->prop_values[i];
			found = 0;
		}
		drmModeFreeProperty(prop);
	}

	drmModeFreeObjectProperties(props);
	return found;
}

// Only the internal panel is calibrated, HDMI output is left untouched.
static int find_panel_crtc(int fd, uint32_t *crtc_id) {
	drmModeRes *res = drmModeGetResources(fd);
	if (!res)
		return -1;

	int found = -1;
	for (int i = 0; i < res->count_connectors && found; i++) {
		drmModeConnector *connector = drmModeGetConnector(fd, res->connectors[i]);
		if (!connector)
			continue;
		if (connector->connector_type == DRM_MODE_CONNECTOR_DSI
			&& connector->connection == DRM_MODE_CONNECTED
			&& connector->encoder_id) {
			drmModeEncoder *encoder = drmModeGetEncoder(fd, connector->encoder_id);
			if (encoder) {
				if (encoder->crtc_id) {
					*crtc_id = encoder->crtc_id;
					found = 0;
				}
				drmModeFreeEncoder(encoder);
			}
		}
		drmModeFreeConnector(connector);
	}

	drmModeFreeResources(res);
	return found;
}

static int open_target(DisplayCalTarget *target) {
	for (int i = 0; i < DISPLAYCAL_CARD_COUNT; i++) {
		char path[32];
		snprintf(path, sizeof(path), "/dev/dri/card%d", i);

		int fd = open(path, O_RDWR);
		if (fd < 0)
			continue;

		uint64_t lut_size = 0;
		if (find_panel_crtc(fd, &target->crtc_id) == 0
			&& find_crtc_prop(fd, target->crtc_id, "GAMMA_LUT", &target->lut_prop, NULL) == 0
			&& find_crtc_prop(fd, target->crtc_id, "GAMMA_LUT_SIZE", NULL, &lut_size) == 0
			&& lut_size > 0) {
			target->fd = fd;
			target->lut_size = (uint32_t)lut_size;
			return 0;
		}

		close(fd);
	}

	fprintf(stderr, "displaycal: no gamma capable panel found\n");
	return -1;
}

static int apply_gains(const DisplayCalGains *gains) {
	DisplayCalTarget target;
	if (open_target(&target) < 0)
		return -1;

	int ret = -1;
	struct drm_color_lut *table = calloc(target.lut_size, sizeof(*table));
	if (table) {
		fill_linear_gain_table(table, target.lut_size, gains);

		uint32_t blob = 0;
		if (drmModeCreatePropertyBlob(target.fd, table, target.lut_size * sizeof(*table), &blob) == 0) {
			ret = drmModeObjectSetProperty(target.fd, target.crtc_id, DRM_MODE_OBJECT_CRTC, target.lut_prop, blob);
			if (ret < 0)
				fprintf(stderr, "displaycal: set gamma table failed: %s\n", strerror(errno));
		}
		else {
			fprintf(stderr, "displaycal: create gamma table failed: %s\n", strerror(errno));
		}

		free(table);
	}

	// the driver keeps the uploaded table, the blob is released with the fd
	close(target.fd);
	return ret;
}

int DisplayCal_enableWithValues(int red_gain, int green_gain, int blue_gain) {
	DisplayCalGains gains = {
		.red_gain = (double)DisplayCal_clampGainValue(red_gain) / DISPLAYCAL_GAIN_SCALE,
		.green_gain = (double)DisplayCal_clampGainValue(green_gain) / DISPLAYCAL_GAIN_SCALE,
		.blue_gain = (double)DisplayCal_clampGainValue(blue_gain) / DISPLAYCAL_GAIN_SCALE,
	};
	return apply_gains(&gains);
}

int DisplayCal_disable(void) {
	// gamma correction stays enabled in hardware once a table has been
	// uploaded, so neutral gains are what actually turns the correction off
	DisplayCalGains gains = { 1.0, 1.0, 1.0 };
	return apply_gains(&gains);
}
