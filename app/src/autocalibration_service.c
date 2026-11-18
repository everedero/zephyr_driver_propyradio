/*
 * Copyright (c) 2026 Philippe Peurichard
 * SPDX-License-Identifier: Apache-2.0
 */

#include "autocalibration_service.h"

#include <errno.h>
#include <limits.h>
#include <stddef.h>
#include "actions.h"
#include "ui.h"

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(autocalibration_service, CONFIG_LOG_DEFAULT_LEVEL);

static lv_anim_t bar_anim;

struct autocalibration_service_state {
	struct k_work_delayable timeout_work;
	struct k_mutex lock;
	struct autocalibration_result result;
	bool active;
	bool complete;
};

static struct autocalibration_service_state service;
static const char *const channel_names[AUTOCALIBRATION_CHANNEL_COUNT] = {
	"ROULIS", "TANGAGE", "GAZ", "LACET",
};

static void set_bar_value(void *bar, int32_t value)
{
	lv_bar_set_value((lv_obj_t *)bar, value, LV_ANIM_ON);
}

static void initialize_bar_anim(lv_anim_t *anim, lv_obj_t *bar) {
    lv_anim_init(anim);
    lv_anim_set_exec_cb(anim, set_bar_value);
    lv_anim_set_duration(anim, AUTOCALIBRATION_DURATION_MS);
    lv_anim_set_var(anim, bar);
    lv_anim_set_values(anim, 0, 100);
}

static void reset_result(struct autocalibration_result *result)
{
	for (size_t i = 0U; i < AUTOCALIBRATION_CHANNEL_COUNT; i++) {
		result->min[i] = UINT16_MAX;
		result->max[i] = 0U;
		result->valid[i] = false;
	}
}

static void autocalibration_timeout(struct k_work *work)
{
	struct k_work_delayable *delayable = k_work_delayable_from_work(work);
	struct autocalibration_service_state *state =
		CONTAINER_OF(delayable, struct autocalibration_service_state, timeout_work);
	struct autocalibration_result result;

	k_mutex_lock(&state->lock, K_FOREVER);
	if (!state->active) {
		k_mutex_unlock(&state->lock);
		return;
	}

	state->active = false;
	state->complete = true;
	result = state->result;
	k_mutex_unlock(&state->lock);

	for (size_t i = 0U; i < AUTOCALIBRATION_CHANNEL_COUNT; i++) {
		if (result.valid[i]) {
			LOG_INF("Autocalibration %s: min=%u max=%u", channel_names[i],
				result.min[i], result.max[i]);
		} else {
			LOG_WRN("Autocalibration %s: no samples captured", channel_names[i]);
		}
	}

	lv_anim_del(&bar_anim, set_bar_value);
	lv_bar_set_value(objects.calib_bar, 100 , LV_ANIM_OFF);
	lv_label_set_text(objects.calib_button_label, "Completed");
}

void action_calib_button_pressed(lv_event_t *e) {
	UNUSED(e);
	if (service.active == true) {
		autocalibration_service_discard();
		lv_anim_del(&bar_anim, set_bar_value);
		lv_bar_set_value(objects.calib_bar, 0 , LV_ANIM_OFF);
		lv_label_set_text(objects.calib_button_label, "Start");
	} else {
		autocalibration_service_start();
		lv_bar_set_value(objects.calib_bar, 0, LV_ANIM_ON);
		initialize_bar_anim(&bar_anim, objects.calib_bar);
		lv_anim_start(&bar_anim);
		lv_label_set_text(objects.calib_button_label, "Stop");
	}
}

int autocalibration_service_init(void)
{
	k_mutex_init(&service.lock);
	reset_result(&service.result);
	service.active = false;
	service.complete = false;
	k_work_init_delayable(&service.timeout_work, autocalibration_timeout);

	return 0;
}

int autocalibration_service_start(void)
{
	int rc;

	k_mutex_lock(&service.lock, K_FOREVER);
	if (service.active) {
		k_mutex_unlock(&service.lock);
		return -EALREADY;
	}
	if (service.complete) {
		k_mutex_unlock(&service.lock);
		return -EBUSY;
	}

	reset_result(&service.result);
	rc = k_work_reschedule(&service.timeout_work,
			       K_MSEC(AUTOCALIBRATION_DURATION_MS));
	if (rc < 0) {
		k_mutex_unlock(&service.lock);
		return rc;
	}

	service.active = true;
	k_mutex_unlock(&service.lock);

	LOG_INF("Autocalibration started for %u seconds",
		AUTOCALIBRATION_DURATION_MS / 1000U);
	return 0;
}

bool autocalibration_service_is_active(void)
{
	bool active;

	k_mutex_lock(&service.lock, K_FOREVER);
	active = service.active;
	k_mutex_unlock(&service.lock);

	return active;
}

int autocalibration_service_observe(index_name_t channel, uint16_t sample)
{
	if ((unsigned int)channel >= AUTOCALIBRATION_CHANNEL_COUNT) {
		return 0;
	}

	k_mutex_lock(&service.lock, K_FOREVER);
	if (service.active) {
		size_t index = (size_t)channel;

		if (!service.result.valid[index] || sample < service.result.min[index]) {
			service.result.min[index] = sample;
		}
		if (!service.result.valid[index] || sample > service.result.max[index]) {
			service.result.max[index] = sample;
		}
		service.result.valid[index] = true;
	}
	k_mutex_unlock(&service.lock);

	return 0;
}

int autocalibration_service_get_result(struct autocalibration_result *result)
{
	if (result == NULL) {
		return -EINVAL;
	}

	k_mutex_lock(&service.lock, K_FOREVER);
	if (!service.complete) {
		k_mutex_unlock(&service.lock);
		return -EAGAIN;
	}

	*result = service.result;
	k_mutex_unlock(&service.lock);

	return 0;
}

static void clear_result(void)
{
	k_mutex_lock(&service.lock, K_FOREVER);
	service.active = false;
	service.complete = false;
	reset_result(&service.result);
	(void)k_work_cancel_delayable(&service.timeout_work);
	k_mutex_unlock(&service.lock);
}

void autocalibration_service_discard(void)
{
	clear_result();
}

void autocalibration_service_clear_result(void)
{
	clear_result();
}
