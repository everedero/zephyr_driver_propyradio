/*
 * model.h
 *
 * Simple model management for RC remote controller application.
 * A "model" is a named set of settings (opaque byte buffer).
 * Supports up to MODEL_MAX_COUNT models.
 */
#ifndef RC_REMOTE_CONTROLLER_MODEL_H
#define RC_REMOTE_CONTROLLER_MODEL_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "platform.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Configuration */
#ifndef MODEL_MAX_COUNT
#define MODEL_MAX_COUNT 6
#endif

#ifndef MODEL_NAME_MAX
#define MODEL_NAME_MAX 32
#endif

/* Channel mapping function type and structure
 * These match the application's channel mapping concept and allow
 * models to store per-channel mapping settings directly.
 */
typedef uint8_t (*map_t)(
    const uint16_t min,
    const uint16_t max,
    const uint16_t center,
          uint16_t data,
    const uint8_t  min_end_point,
    const uint8_t  max_end_point,
    const bool is_reversed);

struct channel_map {
    uint16_t min;
    uint16_t max;
    uint16_t center;
    uint16_t input;
    uint8_t min_end_point;
    uint8_t max_end_point;
    bool is_reversed;
    map_t map;
};

/* Model stores a named `struct channel_map` as its payload. */
typedef struct {
    bool used;                          /* slot in use */
    char name[MODEL_NAME_MAX];          /* NUL-terminated name */
    struct channel_map ch[MAX_CHANNELS];              /* channel mapping data */
} model_t;

/* The persistent model list is placed in a dedicated linker section.
 * Using `.rodata.model_data` keeps the array in read-only flash storage,
 * and __attribute__((used)) prevents the linker from discarding it.
 */
extern model_t active_model;


extern uint8_t def_map(
	const uint16_t min,
	const uint16_t max,
	const uint16_t center,
	      uint16_t data,
    const uint8_t  min_end_point,
    const uint8_t  max_end_point,
	const bool is_reversed);

/* Initialize UI active model & UI model list.
 * Returns 0 on success, error code on error.
 */
int model_init(void);

/* Add a new model with the given name .
 * Returns 0 or error code.
 */
int model_add(const char *name);

/* Store active model in persistent storage.
 * Returns 0 on success, error code on error
 */
int model_store(const struct channel_map *map);

/*
 * Apply four joystick calibration ranges to the active model and persist it.
 * The caller must ensure every axis has a captured range and refresh any
 * separate runtime channel-map copy after success.
 */
int model_apply_calibration(const uint16_t min[CALIBRATION_AXIS_COUNT],
                            const uint16_t max[CALIBRATION_AXIS_COUNT]);

/*
 * Restore the active model's factory calibration ranges and persist it.
 * The caller must refresh any separate runtime channel-map copy after success.
 */
int model_reset_calibration(void);

/* Remove a model based on its name. 
 * Returns 0 on success, error code on error.
 */
int model_remove(char *model_name);

/* Load a model and make it the active model.
 * return 0 or error code.
 */
int model_load(const char *model_name);

int model_recover_during_init(void);

#ifdef __cplusplus
}
#endif

#endif /* RC_REMOTE_CONTROLLER_MODEL_H */
