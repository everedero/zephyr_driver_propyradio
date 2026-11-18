/*
 * Copyright (c) 2026 Philippe Peurichard
 * SPDX-License-Identifier: Apache-2.0
 */

#include "display_services.h"

#include <errno.h>

#include <zephyr/drivers/display.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(display_services, CONFIG_LOG_DEFAULT_LEVEL);

static int schedule_timeout_locked(struct display_idle_service *service)
{
	int rc = k_work_reschedule(&service->timeout_work,
				   K_MSEC(service->inactivity_timeout_ms));

	return rc < 0 ? rc : 0;
}

static void display_idle_timeout(struct k_work *work)
{
	struct k_work_delayable *delayable = k_work_delayable_from_work(work);
	struct display_idle_service *service =
		CONTAINER_OF(delayable, struct display_idle_service, timeout_work);
	int rc;

	k_mutex_lock(&service->lock, K_FOREVER);
	if (!service->running || service->state == DISPLAY_IDLE_STATE_OFF) {
		k_mutex_unlock(&service->lock);
		return;
	}

	rc = display_blanking_on(service->display);
	
	service->state = DISPLAY_IDLE_STATE_OFF;
	LOG_INF("Display panel blanked after inactivity");

	k_mutex_unlock(&service->lock);
}

int display_idle_service_init(struct display_idle_service *service,
			      const struct device *display,
			      uint32_t inactivity_timeout_ms)
{
	if (service == NULL || display == NULL || inactivity_timeout_ms == 0U) {
		return -EINVAL;
	}
	if (!device_is_ready(display)) {
		return -ENODEV;
	}

	service->display = display;
	service->inactivity_timeout_ms = inactivity_timeout_ms;
	service->state = DISPLAY_IDLE_STATE_ON;
	service->running = false;
	k_mutex_init(&service->lock);
	k_work_init_delayable(&service->timeout_work, display_idle_timeout);

	return 0;
}

int display_idle_service_start(struct display_idle_service *service)
{
	int rc;

	if (service == NULL || service->display == NULL) {
		return -EINVAL;
	}

	k_mutex_lock(&service->lock, K_FOREVER);
	service->running = true;
	rc = schedule_timeout_locked(service);
	if (rc < 0) {
		service->running = false;
	}
	k_mutex_unlock(&service->lock);

	return rc;
}

int display_idle_service_note_activity(struct display_idle_service *service)
{
	int rc;

	if (service == NULL || service->display == NULL) {
		return -EINVAL;
	}

	k_mutex_lock(&service->lock, K_FOREVER);
	if (!service->running) {
		k_mutex_unlock(&service->lock);
		return -EACCES;
	}

	rc = service->state == DISPLAY_IDLE_STATE_ON
		     ? schedule_timeout_locked(service)
		     : 0;
	k_mutex_unlock(&service->lock);

	return rc;
}

int display_idle_service_stop(struct display_idle_service *service)
{
	struct k_work_sync sync;

	if (service == NULL || service->display == NULL) {
		return -EINVAL;
	}

	k_mutex_lock(&service->lock, K_FOREVER);
	service->running = false;
	k_mutex_unlock(&service->lock);

	(void)k_work_cancel_delayable_sync(&service->timeout_work, &sync);
	return 0;
}

int display_touch_wake_service_init(
	struct display_touch_wake_service *service,
	struct display_idle_service *idle_service)
{
	if (service == NULL || idle_service == NULL ||
	    idle_service->display == NULL) {
		return -EINVAL;
	}

	service->idle_service = idle_service;
	service->main_screen_requested = false;
	return 0;
}

int display_touch_wake_service_on_touch(
	struct display_touch_wake_service *service)
{
	struct display_idle_service *idle_service;
	int rc = 0;

	if (service == NULL || service->idle_service == NULL) {
		return -EINVAL;
	}

	idle_service = service->idle_service;
	k_mutex_lock(&idle_service->lock, K_FOREVER);
	if (!idle_service->running) {
		k_mutex_unlock(&idle_service->lock);
		return -EACCES;
	}

	if (idle_service->state == DISPLAY_IDLE_STATE_OFF) {
		rc = display_blanking_off(idle_service->display);
		if (rc < 0) {
			LOG_ERR("Failed to restore display (%d)", rc);
			k_mutex_unlock(&idle_service->lock);
			return rc;
		}
		idle_service->state = DISPLAY_IDLE_STATE_ON;
		service->main_screen_requested = true;
		LOG_INF("Touchscreen restored display panel");
	}

	rc = schedule_timeout_locked(idle_service);
	k_mutex_unlock(&idle_service->lock);

	return rc;
}

bool display_touch_wake_service_take_main_screen_request(
	struct display_touch_wake_service *service)
{
	bool requested;

	if (service == NULL || service->idle_service == NULL) {
		return false;
	}

	k_mutex_lock(&service->idle_service->lock, K_FOREVER);
	requested = service->main_screen_requested;
	service->main_screen_requested = false;
	k_mutex_unlock(&service->idle_service->lock);

	return requested;
}
