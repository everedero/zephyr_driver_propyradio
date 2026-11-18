/*
 * Copyright (c) 2026 Philippe Peurichard
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef AUTOCALIBRATION_SERVICE_H
#define AUTOCALIBRATION_SERVICE_H

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef __cplusplus
extern "C" {
#endif

#define AUTOCALIBRATION_DURATION_MS 30000U
#define AUTOCALIBRATION_CHANNEL_COUNT CALIBRATION_AXIS_COUNT

struct autocalibration_result {
	uint16_t min[AUTOCALIBRATION_CHANNEL_COUNT];
	uint16_t max[AUTOCALIBRATION_CHANNEL_COUNT];
	bool valid[AUTOCALIBRATION_CHANNEL_COUNT];
};

/* Initialize once before starting threads that use the service. */
int autocalibration_service_init(void);

/* Start a 30-second capture; returns -EALREADY or -EBUSY if not available. */
int autocalibration_service_start(void);
bool autocalibration_service_is_active(void);

/* Record a sample while active; non-joystick channels are ignored. */
int autocalibration_service_observe(index_name_t channel, uint16_t sample);

/* Read completed candidate values; returns -EAGAIN until the timer expires. */
int autocalibration_service_get_result(struct autocalibration_result *result);

/* Stop and discard capture/results, or clear a result after it has been used. */
void autocalibration_service_discard(void);
void autocalibration_service_clear_result(void);

#ifdef __cplusplus
}
#endif

#endif /* AUTOCALIBRATION_SERVICE_H */
