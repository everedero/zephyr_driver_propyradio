/*
 * Copyright (c) 2026 Philippe Peurichard
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef DISPLAY_SERVICES_H
#define DISPLAY_SERVICES_H

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/kernel.h>

#define DISPLAY_IDLE_DEFAULT_TIMEOUT_MS 30000U

#ifdef __cplusplus
extern "C" {
#endif

enum display_idle_state {
	DISPLAY_IDLE_STATE_ON,
	DISPLAY_IDLE_STATE_OFF,
};

struct display_idle_service {
	const struct device *display;
	struct k_work_delayable timeout_work;
	struct k_mutex lock;
	uint32_t inactivity_timeout_ms;
	enum display_idle_state state;
	bool running;
};

struct display_touch_wake_service {
	struct display_idle_service *idle_service;
	bool main_screen_requested;
};

/**
 * @brief Initialize the inactivity timeout service.
 *
 * @param service Service state to initialize.
 * @param display Zephyr display device, normally DEVICE_DT_GET(DT_CHOSEN(zephyr_display)).
 * @param inactivity_timeout_ms Idle time before the panel is blanked.
 * @return 0 on success, or a negative error code.
 */
int display_idle_service_init(struct display_idle_service *service,
			      const struct device *display,
			      uint32_t inactivity_timeout_ms);

/**
 * @brief Start the inactivity timer.
 *
 * When the timeout expires, the service blanks the panel using the Zephyr
 * display API. The touch controller is not stopped or powered down.
 *
 * @param service Initialized service.
 * @return 0 on success, or a negative error code.
 */
int display_idle_service_start(struct display_idle_service *service);

/**
 * @brief Restart the inactivity timer after non-touch user activity.
 *
 * Does not turn the display back on if it is already blanked.
 *
 * @param service Initialized, running service.
 * @return 0 on success, or a negative error code.
 */
int display_idle_service_note_activity(struct display_idle_service *service);

/**
 * @brief Stop the inactivity timer.
 *
 * @param service Initialized service.
 * @return 0 on success, or a negative error code.
 */
int display_idle_service_stop(struct display_idle_service *service);

/**
 * @brief Initialize touchscreen wake handling for an idle service.
 *
 * @param service Touch wake service state to initialize.
 * @param idle_service Initialized inactivity service to control.
 * @return 0 on success, or a negative error code.
 */
int display_touch_wake_service_init(
	struct display_touch_wake_service *service,
	struct display_idle_service *idle_service);

/**
 * @brief Handle a touchscreen contact-down event.
 *
 * Call from the FT5336/Zephyr input callback on touch-down events only.
 * While the panel is blanked, this restores it through the Zephyr display
 * API and requests SCREEN_ID_MAIN; the UI thread must consume that request.
 * While it is on, the touch restarts the inactivity timer.
 *
 * @param service Initialized touch wake service.
 * @return 0 on success, or a negative error code.
 */
int display_touch_wake_service_on_touch(
	struct display_touch_wake_service *service);

/**
 * @brief Consume a pending request to load the main screen.
 *
 * Call from the UI-owning thread so LVGL screen changes are not made from the
 * Zephyr input thread.
 *
 * @param service Initialized touch wake service.
 * @return true if the caller should load SCREEN_ID_MAIN.
 */
bool display_touch_wake_service_take_main_screen_request(
	struct display_touch_wake_service *service);

#ifdef __cplusplus
}
#endif

#endif /* DISPLAY_SERVICES_H */
